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

static FONT: [u8; 480] =
[
	0x00,0x00,0x00,0x00,0x00, // - -
	0x00,0x00,0x5F,0x00,0x00, // -!-
	0x00,0x07,0x00,0x07,0x00, // -"-
	0x14,0x7F,0x14,0x7F,0x14, // -#-
	0x24,0x2E,0x7B,0x2A,0x12, // -$-
	0x23,0x13,0x08,0x64,0x62, // -%-
	0x36,0x49,0x56,0x20,0x50, // -&-
	0x00,0x04,0x03,0x01,0x00, // -'-
	0x00,0x1C,0x22,0x41,0x00, // -(-
	0x00,0x41,0x22,0x1C,0x00, // -)-
	0x22,0x14,0x7F,0x14,0x22, // -*-
	0x08,0x08,0x7F,0x08,0x08, // -+-
	0x40,0x30,0x10,0x00,0x00, // -,-
	0x08,0x08,0x08,0x08,0x08, // ---
	0x00,0x60,0x60,0x00,0x00, // -.-
	0x20,0x10,0x08,0x04,0x02, // -/-
	0x3E,0x51,0x49,0x45,0x3E, // -0-
	0x00,0x42,0x7F,0x40,0x00, // -1-
	0x62,0x51,0x49,0x49,0x46, // -2-
	0x21,0x41,0x49,0x4D,0x33, // -3-
	0x18,0x14,0x12,0x7F,0x10, // -4-
	0x27,0x45,0x45,0x45,0x39, // -5-
	0x3C,0x4A,0x49,0x49,0x31, // -6-
	0x01,0x71,0x09,0x05,0x03, // -7-
	0x36,0x49,0x49,0x49,0x36, // -8-
	0x46,0x49,0x49,0x29,0x1E, // -9-
	0x00,0x36,0x36,0x00,0x00, // -:-
	0x40,0x36,0x36,0x00,0x00, // -;-
	0x08,0x14,0x22,0x41,0x00, // -<-
	0x14,0x14,0x14,0x14,0x14, // -=-
	0x00,0x41,0x22,0x14,0x08, // ->-
	0x02,0x01,0x59,0x05,0x02, // -?-
	0x3E,0x41,0x5D,0x55,0x5E, // -@-
	0x7C,0x12,0x11,0x12,0x7C, // -A-
	0x7F,0x49,0x49,0x49,0x36, // -B-
	0x3E,0x41,0x41,0x41,0x22, // -C-
	0x7F,0x41,0x41,0x41,0x3E, // -D-
	0x7F,0x49,0x49,0x49,0x41, // -E-
	0x7F,0x09,0x09,0x09,0x01, // -F-
	0x3E,0x41,0x51,0x51,0x72, // -G-
	0x7F,0x08,0x08,0x08,0x7F, // -H-
	0x00,0x41,0x7F,0x41,0x00, // -I-
	0x20,0x40,0x41,0x3F,0x01, // -J-
	0x7F,0x08,0x14,0x22,0x41, // -K-
	0x7F,0x40,0x40,0x40,0x40, // -L-
	0x7F,0x02,0x0C,0x02,0x7F, // -M-
	0x7F,0x04,0x08,0x10,0x7F, // -N-
	0x3E,0x41,0x41,0x41,0x3E, // -O-
	0x7F,0x09,0x09,0x09,0x06, // -P-
	0x3E,0x41,0x51,0x21,0x5E, // -Q-
	0x7F,0x09,0x19,0x29,0x46, // -R-
	0x26,0x49,0x49,0x49,0x32, // -S-
	0x01,0x01,0x7F,0x01,0x01, // -T-
	0x3F,0x40,0x40,0x40,0x3F, // -U-
	0x1F,0x20,0x40,0x20,0x1F, // -V-
	0x7F,0x20,0x18,0x20,0x7F, // -W-
	0x63,0x14,0x08,0x14,0x63, // -X-
	0x03,0x04,0x78,0x04,0x03, // -Y-
	0x61,0x51,0x49,0x45,0x43, // -Z-
	0x7F,0x7F,0x41,0x41,0x00, // -[-
	0x02,0x04,0x08,0x10,0x20, // -\-
	0x00,0x41,0x41,0x7F,0x7F, // -]-
	0x04,0x02,0x7F,0x02,0x04, // -^-
	0x08,0x1C,0x2A,0x08,0x08, // -_-
	0x00,0x00,0x01,0x02,0x04, // -`-
	0x24,0x54,0x54,0x38,0x40, // -a-
	0x7F,0x28,0x44,0x44,0x38, // -b-
	0x38,0x44,0x44,0x44,0x08, // -c-
	0x38,0x44,0x44,0x28,0x7F, // -d-
	0x38,0x54,0x54,0x54,0x08, // -e-
	0x08,0x7E,0x09,0x09,0x02, // -f-
	0x98,0xA4,0xA4,0xA4,0x78, // -g-
	0x7F,0x08,0x04,0x04,0x78, // -h-
	0x00,0x00,0x79,0x00,0x00, // -i-
	0x00,0x80,0x88,0x79,0x00, // -j-
	0x7F,0x10,0x28,0x44,0x40, // -k-
	0x00,0x41,0x7F,0x40,0x00, // -l-
	0x78,0x04,0x78,0x04,0x78, // -m-
	0x04,0x78,0x04,0x04,0x78, // -n-
	0x38,0x44,0x44,0x44,0x38, // -o-
	0xFC,0x24,0x24,0x24,0x18, // -p-
	0x18,0x24,0x24,0x24,0xFC, // -q-
	0x04,0x78,0x04,0x04,0x08, // -r-
	0x48,0x54,0x54,0x54,0x24, // -s-
	0x04,0x3F,0x44,0x44,0x24, // -t-
	0x3C,0x40,0x40,0x3C,0x40, // -u-
	0x1C,0x20,0x40,0x20,0x1C, // -v-
	0x3C,0x40,0x3C,0x40,0x3C, // -w-
	0x44,0x28,0x10,0x28,0x44, // -x-
	0x9C,0xA0,0xA0,0x90,0x7C, // -y-
	0x44,0x64,0x54,0x4C,0x44, // -z-
	0x08,0x36,0x41,0x00,0x00, // -{-
	0x00,0x00,0x77,0x00,0x00, // -|-
	0x00,0x00,0x41,0x36,0x08, // -}-
	0x08,0x04,0x08,0x10,0x08, // -~-
	0x55,0x2A,0x55,0x2A,0x55, // -
    ];
pub struct St7585<RST, WS, D, SPI> {
    spi: SPI,
    rst: RST,
    ws: WS,
    delay: D,
    x: u8,
    y: u8,
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

        let _ = self.clear_screen();

        Ok(())
    }

    pub fn clear_screen(&mut self) -> Result<(), E> {
        let _ = self.set_xy(0, 0);
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
        self.x = 0;
        self.y = 7;
        for c in s.chars() {
            self.write_char(c)?;
        }
        self.delay.delay_ms(1);
        Ok(())
    }

    fn write_char(&mut self, c: char) -> fmt::Result {
        self.x = self.x + 6 as u8;
        if self.x >= 96 as u8 {
            self.x = 0 as u8;
            self.y = self.y - 1 as u8;
            if self.y == 0 as u8 {
                self.y = 7 as u8;
            }
        }
        let _ = self.ws.set_low();
        let _ = self.spi.write(&[0x40 as u8 | self.y as u8]);
        let _ = self.spi.write(&[0x80 as u8 | self.x as u8]);
        let _ = self.ws.set_high();
        let index = ((c as u16 - 32 as u16) * 5) as usize;
        if index > 480 as usize {
            return Ok(())
        }
        //info!("x {}, y {}, c {}, index {}", self.x, self.y, c, index);
        for i in 0..5 {
            let _ = self.spi.write(&[FONT[index + i as usize]]);
        }
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
            if str_resp.contains("ok") || str_resp.contains("ERR") {
                info!("{}", str_resp);
                break;
            }
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
    writeln!(st7585, "Hello St7585, Nice to meet you").unwrap();
    // reset usr_wifi232_t
    rst.set_low();
    Timer::after_millis(300).await;
    rst.set_high();
    Timer::after_millis(3000).await;
    info!("111");
    // enter at command mode
    let mut s = [0u8; 128];
    unwrap!(usr_tx.write_all("+".as_bytes()).await);
    Timer::after_millis(100).await;
    unwrap!(usr_tx.write_all("+".as_bytes()).await);
    Timer::after_millis(100).await;
    unwrap!(usr_tx.write_all("+".as_bytes()).await);
    Timer::after_millis(100).await;
    unwrap!(usr_rx.read(&mut s).await);
    unwrap!(usr_tx.write_all("a".as_bytes()).await);
    unwrap!(usr_rx.read(&mut s).await);
    info!("222");
    // waiting finish
    Timer::after_millis(1000).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wmode=sta\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wmode=sta\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wsssid\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wskey\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+netp=TCP,Server,1234,192.168.1.10\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+tcpdis=on\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+httptp=get\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+httpurl=123.56.216.251,443\r", &mut s).await;
    //usr_cmd(&mut usr_rx, &mut usr_tx, "at+httpurl=https://devapi.qweather.com,443\r", &mut s).await;
    //usr_cmd(&mut usr_rx, &mut usr_tx, "at+httpurl=https://devapi.qweather.com/v7/minutely/5m?location=39.95,116.46\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+httpph=/v7/minutely/5m?location=39.95,116.46&key=c8cd8ac05fcb4808baf95c58c94c2fe8\r", &mut s).await;
    //usr_cmd(&mut usr_rx, &mut usr_tx, "at+httpph=X-QW-Api-Key: c8cd8ac05fcb4808baf95c58c94c2fe8\r", &mut s).await;

    loop {
        let mut ss = [0u8; 512];
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+wann\r", &mut s).await;
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+netp\r", &mut s).await;
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+tcplk\r", &mut s).await;
        let tcplk = core::str::from_utf8(&s).unwrap();
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+httpdt\r", &mut ss).await;
        let ping = core::str::from_utf8(&ss).unwrap();
        let _ = st7585.clear_screen();
        writeln!(st7585, "{ping}").unwrap();
        if ping.contains("Success") && tcplk.contains("on") {
            info!("network stable!");
            usr_cmd(&mut usr_rx, &mut usr_tx, "at+entm\r", &mut s).await;
            break;
        }
        Timer::after_millis(2000).await;
    }
}
