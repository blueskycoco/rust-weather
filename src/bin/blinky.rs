#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::usart::{Config, BufferedUart, BufferedUartRx, BufferedUartTx};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_stm32::gpio::{Level, Output, Speed, Pin, AnyPin};
use embassy_time::{Timer, Delay};
use embassy_stm32::time::Hertz;
use embedded_io_async::BufRead;
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Write as SpiWrite};
use embedded_hal::blocking::delay::DelayMs;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use core::fmt;
use core::fmt::Write as FmtWrite;
pub struct St7585<RST, WS, D, SPI> {
    spi: SPI,
    rst: RST,
    ws: WS,
    delay: D,
    x: u16,
    y: u16,
}

impl <RST, WS, D, SPI, E> St7585<RST, WS, D, SPI>
    where
        RST: OutputPin,
        WS: OutputPin,
        D: DelayMs<u8>,
        SPI: SpiWrite<u8, Error = E>,
{
    pub fn new(
        rst: RST,
        ws: WS,
        delay: D,
        spi: SPI) -> Self {
        Self {spi, rst, ws, delay, x: 0, y: 7}
    }

    pub fn init(&mut self) -> Result<(), E> {
        let _ = self.rst.set_low();
        let _ = self.delay.delay_ms(100);
        let _ = self.rst.set_high();
        let _ = self.delay.delay_ms(100);
        
        let _ = self.cmd(0x21, true);
        let _ = self.cmd(0x9c, true);
        let _ = self.cmd(0x30, true);
        let _ = self.cmd(0x20, true);
        let _ = self.cmd(0x0c, true);
        
        //clear screen
        let _ = self.cmd(0x40, true);
        let _ = self.cmd(0x80, true);
        for _ in 0..960 {
            let _ = self.cmd(0x00, false);
        }

        let _ = self.set_xy(95, 8);
        let _ = self.cmd(0xff, false);
        let _ = self.set_xy(94, 8);
        let _ = self.cmd(0xff, false);
        let _ = self.set_xy(93, 8);
        let _ = self.cmd(0xff, false);
        let _ = self.set_xy(80, 8);
        let _ = self.cmd(0xff, false);
        Ok(())
    }

    fn set_xy(&mut self, x: u8, y: u8) -> Result<(), E> {
        let _ = self.cmd(0x40 | y, true);
        self.cmd(0x80 | x, true)
    }

    fn cmd(&mut self, command: u8, dc: bool) -> Result<(), E> {
        if dc {
            let _ = self.ws.set_low();
        } else {
            let _ = self.ws.set_high();
        }

        self.spi.write(&[command])
    }
}

impl<RST, WS, D, SPI, E> fmt::Write for St7585<RST, WS, D, SPI>
where
    SPI: SpiWrite<u8, Error = E>,
    D: DelayMs<u8>,
    WS: OutputPin,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            self.write_char(c)?;
        }
        self.delay.delay_ms(1);
        Ok(())
    }

    fn write_char(&mut self, c: char) -> fmt::Result {
        let mut command = [0x40, 0, 0, 0];
        c.encode_utf8(&mut command[1..]);
        self.x = self.x + 6 as u16;
        if self.x >= 96 as u16 {
            self.x = 0 as u16;
            self.y = self.y - 1 as u16;
            if self.y == 0 as u16 {
                self.y = 7 as u16;
            }
        }
        let _ = self.ws.set_low();
        let _ = self.spi.write(&[0x40 as u8 | self.y as u8]);
        let _ = self.spi.write(&[0x80 as u8 | self.x as u8]);
        let _ = self.ws.set_high();
        let _ = self.spi.write(&command[..2]);
        Ok(())
    }
}

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

fn clear(ary: &mut [u8]) {
    ary.iter_mut().for_each(|m| *m = 0)
}

#[embassy_executor::task]
async fn blinky(pin: AnyPin) {
    let mut led = Output::new(pin, Level::High, Speed::Low);

    loop {
        led.set_high();
        Timer::after_millis(300).await;

        led.set_low();
        Timer::after_millis(300).await;
    }
}

async fn usr_cmd(rx: &mut BufferedUartRx<'_>,
                    tx: &mut BufferedUartTx<'_>,
                    cmd: &str,
                    s: &mut [u8]) {
    clear(s);
    unwrap!(tx.write_all(cmd.as_bytes()).await);
    let mut cnt = 0;
    loop {
        let n = rx.read(&mut s[cnt..]).await;
        match n {
            Ok(bytes) => cnt = cnt + bytes,
            Err(e) => info!("read error {}", e),
        }

        if s.get(cnt - 4) == Some(&b'\r') && s.get(cnt - 3) == Some(&b'\n') &&
           s.get(cnt - 2) == Some(&b'\r') && s.get(cnt - 1) == Some(&b'\n') {
            let str_resp = core::str::from_utf8(s).unwrap();
            info!("{}", str_resp);
            break;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            // Oscillator for bluepill, Bypass for nucleos.
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    let p = embassy_stm32::init(config);
    //let p = embassy_stm32::init(Default::default());

    spawner.spawn(blinky(p.PB14.degrade())).unwrap();
    let mut config = Config::default();
    config.baudrate = 115200;
    static TX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 128])[..];
    static RX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 128])[..];
    let usart = BufferedUart::new(p.USART2, Irqs, p.PA3, p.PA2, tx_buf,
                                      rx_buf, config).unwrap();
    let (mut usr_tx, mut usr_rx) = usart.split();
    let mut rst = Output::new(p.PA0, Level::High, Speed::Low);

    let st7585_rst = Output::new(p.PA1, Level::High, Speed::Low);
    let st7585_ws = Output::new(p.PA8, Level::High, Speed::Low);
    let _st7585_cs = Output::new(p.PA4, Level::Low, Speed::Low);
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(20_000_000);
    let st7585_spi = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_config);

    let mut st7585 = St7585::new(st7585_rst, st7585_ws, Delay, st7585_spi);
    let _ = st7585.init();
    writeln!(st7585, "Hello St7585").unwrap();
    // reset usr_wifi232_t
    Timer::after_millis(200).await;
    rst.set_low();
    Timer::after_millis(300).await;
    rst.set_high();
    Timer::after_millis(2000).await;

    // enter at command mode
    let mut s = [0u8; 128];
    unwrap!(usr_tx.write_all("+++".as_bytes()).await);
    unwrap!(usr_rx.read(&mut s).await);
    unwrap!(usr_tx.write_all("a".as_bytes()).await);
    unwrap!(usr_rx.read(&mut s).await);
    // waiting finish
    Timer::after_millis(200).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wmode=sta\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wmode=sta\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wsssid\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wskey\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+netp=TCP,Server,1234,192.168.1.10\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+tcpdis=on\r", &mut s).await;

    loop {
        let mut ss = [0u8; 128];
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+wann\r", &mut s).await;
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+netp\r", &mut s).await;
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+tcplk\r", &mut s).await;
        let tcplk = core::str::from_utf8(&s).unwrap();
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+ping=www.baidu.com\r", &mut ss).await;
        let ping = core::str::from_utf8(&ss).unwrap();
        if ping.contains("Success") && tcplk.contains("on") {
            info!("network stable!");
            usr_cmd(&mut usr_rx, &mut usr_tx, "at+entm\r", &mut s).await;
            break;
        }
        Timer::after_millis(2000).await;
    }
}
