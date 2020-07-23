#![no_std]
#![no_main]

// Import the right HAL/PAC crate, depending on the target chip
#[cfg(feature = "51")]
use nrf51_hal as hal;
#[cfg(feature = "52810")]
use nrf52810_hal as hal;
#[cfg(feature = "52832")]
use nrf52832_hal as hal;
#[cfg(feature = "52840")]
use nrf52840_hal as hal;

use {
    core::{
        default::Default,
        fmt::Write,
        panic::PanicInfo,
        sync::atomic::{compiler_fence, AtomicBool, Ordering},
    },
    // embedded_hal::blocking::spi::{Transfer, Write},
    esb::{
        consts::*, irq::StatePTX, Addresses, BBBuffer, ConfigBuilder, ConstBBBuffer, Error, EsbApp,
        EsbBuffer, EsbHeader, EsbIrq, IrqTimer,
    },
    hal::{
        gpio::{self, p0, p0::*, Level, PushPull},
        pac::{TIMER0, TIMER1},
        spim::Spim,
    },

    rtt_target::{rprintln, rtt_init_print},
};

use heapless::String;

const MAX_PAYLOAD_SIZE: u8 = 64;

static DELAY_FLAG: AtomicBool = AtomicBool::new(false);
static ATTEMPTS_FLAG: AtomicBool = AtomicBool::new(false);

#[rtic::app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        esb_app: EsbApp<U1024, U1024>,
        esb_irq: EsbIrq<U1024, U1024, TIMER0, StatePTX>,
        esb_timer: IrqTimer<TIMER0>,
        delay: TIMER1,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        let d = ctx.device;

        let _clocks = hal::clocks::Clocks::new(d.CLOCK).enable_ext_hfosc();
        rtt_init_print!();
        rprintln!("PTX init");

        let port0 = p0::Parts::new(d.P0);

        // Who am I

        let mut cs = port0.p0_27.into_push_pull_output(Level::High);
        let spimiso = port0.p0_23.into_floating_input().degrade();
        let spimosi = port0.p0_29.into_push_pull_output(Level::Low).degrade();
        let spiclk = port0.p0_12.into_push_pull_output(Level::Low).degrade();

        let pins = hal::spim::Pins {
            sck: spiclk,
            miso: Some(spimiso),
            mosi: Some(spimosi),
        };

        let mut spi = Spim::new(
            d.SPIM2,
            pins,
            hal::spim::Frequency::K500,
            hal::spim::MODE_3,
            0,
        );

        let mut lsm6 = lsm6::Lsm6::new(spi, cs.degrade()).unwrap();
        rprintln!("WHO_AM_I {}", lsm6.who_am_i().unwrap());

        // CTRL1_XL (Accelerometer)
        // ODR =

        enum OdrXl {
            Hz0 = 0b0000,    // Power-down
            Hz12_5 = 0b0001, // low power
            Hz26 = 0b0010,   // low power
            Hz52 = 0b0011,   // low power
            Hz104 = 0b0100,  // normal mode
            Hz208 = 0b0101,  // normal mode
            Hz416 = 0b0110,  // high performance
            Hz833 = 0b0111,  // high performance
            Hz1660 = 0b1000, // high performance
            Hz3330 = 0b1001, // high performance
            Hz6660 = 0b1010, // high performance
        }
        enum FsXl {
            G2 = 0b00,  // +/- 2g
            G4 = 0b11,  // +/- 4g
            G8 = 0b10,  // +/- 8g
            G16 = 0b01, // +/- 16g
        }
        enum BwXl {
            Hz400,
            Hz200,
            Hz100,
            Hz50,
        }

        lsm6.write_register(
            lsm6::Register::CTRL1_XL,
            ((OdrXl::Hz12_5 as u8) << 4) | ((FsXl::G2 as u8) << 2) | BwXl::Hz50 as u8,
        );

        loop {
            match lsm6.read_register(lsm6::Register::STATUS_REG).unwrap() {
                0 => {}
                data => {
                    rprintln!("status {}", data);
                    // bit ugly, TDA
                    if data & 0x4 == 0x4 {
                        let temp_l = lsm6.read_register(lsm6::Register::OUT_TEMP_L).unwrap();
                        let temp_h = lsm6.read_register(lsm6::Register::OUT_TEMP_H).unwrap();
                        let temp = (((temp_h as u16) << 8) + temp_l as u16) as i16;
                        let temp = (temp as f32 / 16.0) + 25.0;
                        rprintln!("temp = {}", temp);
                    }
                    // bit ugly, XLDA
                    if data & 0x1 == 0x1 {
                        let outx_l = lsm6.read_register(lsm6::Register::OUTX_L_XL).unwrap();
                        let outx_h = lsm6.read_register(lsm6::Register::OUTX_H_XL).unwrap();
                        let outy_l = lsm6.read_register(lsm6::Register::OUTY_L_XL).unwrap();
                        let outy_h = lsm6.read_register(lsm6::Register::OUTY_H_XL).unwrap();
                        let outz_l = lsm6.read_register(lsm6::Register::OUTZ_L_XL).unwrap();
                        let outz_h = lsm6.read_register(lsm6::Register::OUTZ_H_XL).unwrap();

                        let outx = (((outx_h as u16) << 8) + outx_l as u16) as i16;
                        let outy = (((outy_h as u16) << 8) + outy_l as u16) as i16;
                        let outz = (((outz_h as u16) << 8) + outz_l as u16) as i16;
                        rprintln!("x = {}, y = {}, z = {}", outx, outy, outz);
                    }
                }
            };
            cortex_m::asm::delay(1000_000);
        }

        // lsm6.write_register(Lsm6::, data)
        // test_embedded_hal(&spi, &mut cs);

        static BUFFER: EsbBuffer<U1024, U1024> = EsbBuffer {
            app_to_radio_buf: BBBuffer(ConstBBBuffer::new()),
            radio_to_app_buf: BBBuffer(ConstBBBuffer::new()),
            timer_flag: AtomicBool::new(false),
        };
        let addresses = Addresses::default();
        let config = ConfigBuilder::default()
            .maximum_transmit_attempts(0)
            .max_payload_size(MAX_PAYLOAD_SIZE)
            .check()
            .unwrap();

        let (esb_app, esb_irq, esb_timer) = BUFFER
            .try_split(d.TIMER0, d.RADIO, addresses, config)
            .unwrap();

        // setup timer for delay
        let timer = d.TIMER1;
        timer.bitmode.write(|w| w.bitmode()._32bit());

        // 16 Mhz / 2**9 = 31250 Hz
        timer.prescaler.write(|w| unsafe { w.prescaler().bits(9) });
        timer.shorts.modify(|_, w| w.compare0_clear().enabled());
        timer.cc[0].write(|w| unsafe { w.bits(31250u32) });
        timer.events_compare[0].reset();
        timer.intenset.write(|w| w.compare0().set());

        // Clears and starts the counter
        timer.tasks_clear.write(|w| unsafe { w.bits(1) });
        timer.tasks_start.write(|w| unsafe { w.bits(1) });

        init::LateResources {
            esb_app,
            esb_irq: esb_irq.into_ptx(),
            esb_timer,
            delay: timer,
        }
    }

    #[idle(resources = [esb_app])]
    fn idle(ctx: idle::Context) -> ! {
        let mut pid = 0;
        let mut ct_tx: usize = 0;
        let mut ct_rx: usize = 0;
        let mut ct_err: usize = 0;
        loop {
            let esb_header = EsbHeader::build()
                .max_payload(MAX_PAYLOAD_SIZE)
                .pid(pid)
                .pipe(0)
                .no_ack(false)
                .check()
                .unwrap();
            if pid == 3 {
                pid = 0;
            } else {
                pid += 1;
            }

            // Did we receive any packet ?
            if let Some(response) = ctx.resources.esb_app.read_packet() {
                ct_rx += 1;
                let text = core::str::from_utf8(&response[..]).unwrap();
                let rssi = response.get_header().rssi();
                rprintln!(
                    "Received PKG #: {} | Payload: {} | rssi: {}",
                    ct_rx,
                    text,
                    rssi
                );
                response.release();
            }
            ct_tx += 1;
            rprintln!("Sending PKG | tx: {} | err: {}", ct_tx, ct_err);

            let mut packet = ctx.resources.esb_app.grant_packet(esb_header).unwrap();
            let mut s: String<U16> = String::new();
            write!(&mut s, "PTX PKG #{}", ct_tx).unwrap();
            let length = s.as_bytes().len();
            &packet[..length].copy_from_slice(s.as_bytes());
            packet.commit(length);
            ctx.resources.esb_app.start_tx();

            while !DELAY_FLAG.load(Ordering::Acquire) {
                if ATTEMPTS_FLAG.load(Ordering::Acquire) {
                    rprintln!("--- Ack not received {}\r", ct_err);
                    ATTEMPTS_FLAG.store(false, Ordering::Release);
                    ct_err += 1;
                }
            }
            DELAY_FLAG.store(false, Ordering::Release);
        }
    }

    #[task(binds = RADIO, resources = [esb_irq], priority = 3)]
    fn radio(ctx: radio::Context) {
        match ctx.resources.esb_irq.radio_interrupt() {
            Err(Error::MaximumAttempts) => {
                rprintln!("Radio Interrupt MaximumAttempts");
                ATTEMPTS_FLAG.store(true, Ordering::Release);
            }
            Err(e) => panic!("Radio Interrupt Error {:?}", e),
            Ok(_) => rprintln!("Radio Interrupt Ok"),
        }
    }

    #[task(binds = TIMER0, resources = [esb_timer], priority = 3)]
    fn timer0(ctx: timer0::Context) {
        ctx.resources.esb_timer.timer_interrupt();
    }

    #[task(binds = TIMER1, resources = [delay], priority = 1)]
    fn timer1(ctx: timer1::Context) {
        ctx.resources.delay.events_compare[0].reset();
        DELAY_FLAG.store(true, Ordering::Release);
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    rprintln!("{}", info);
    loop {
        compiler_fence(Ordering::SeqCst);
    }
}

mod lsm6 {

    extern crate embedded_hal as hal;
    use embedded_hal::{
        blocking::spi::{Transfer, Write},
        digital::v2::OutputPin,
    };

    /// LSM6 driver
    ///
    /// For now assume mode 3

    pub struct Lsm6<SPI, CS> {
        spi: SPI,
        cs: CS,
    }

    impl<SPI, CS, E> Lsm6<SPI, CS>
    where
        SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
        CS: OutputPin,
    {
        /// Create the driver
        pub fn new(spi: SPI, cs: CS) -> Result<Self, E> {
            // Todo: power up?
            Ok(Lsm6 { spi, cs })
        }

        /// Read specific register
        pub fn read_register(&mut self, reg: Register) -> Result<u8, E> {
            const READ: u8 = 0x80u8;
            self.cs.set_low();
            let mut buffer = [reg.addr() | READ, 0];
            self.spi.transfer(&mut buffer)?;
            self.cs.set_high();
            Ok(buffer[1])
        }

        /// Write specific register
        pub fn write_register(&mut self, reg: Register, data: u8) -> Result<(), E> {
            self.cs.set_low();
            let mut buffer = [reg.addr(), data];
            self.spi.transfer(&mut buffer)?;
            self.cs.set_high();
            Ok(())
        }

        /// Reads the WHO_AM_I register; should return `01101001`
        pub fn who_am_i(&mut self) -> Result<u8, E> {
            self.read_register(Register::WHO_AM_I)
        }

        /// Read registers, not yet implemented
        pub fn read_registers(&mut self, reg: Register) -> Result<u8, E> {
            self.read_register(Register::WHO_AM_I)
        }
    }

    #[allow(dead_code)]
    #[allow(non_camel_case_types)]
    #[derive(Clone, Copy)]
    pub enum Register {
        // Embedded functions configuration register
        FUNC_CFG_ACCESS = 0x01,

        // Sensor sync configuration register
        SENSOR_SYNC_TIME_FRAME = 0x04,

        // FIFO configuration register
        FIFO_CTRL1 = 0x06,
        FIFO_CTRL2 = 0x07,
        FIFO_CTRL3 = 0x08,
        FIFO_CTRL4 = 0x09,
        FIFO_CTRL5 = 0x0A,

        //
        ORIENT_CFG_G = 0x0B,

        // INT1 pin control
        INT1_CTRL = 0x0D,

        // INT2 pin control
        INT2_CTRL = 0x0E,

        // Who I am I ID (default 01101001)
        WHO_AM_I = 0x0F,

        // Accelerometer and gyroscope control registers
        CTRL1_XL = 0x10,
        CTRL2_G = 0x11,
        CTRL3_C = 0x12,
        CTRL4_C = 0x13,
        CTRL5_C = 0x14,
        CTRL6_C = 0x15,
        CTRL7_XL = 0x16,
        CTRL8_XL = 0x17,
        CTRL9_XL = 0x18,
        CTRL10_XL = 0x19,

        // I2C master configuration register
        MASTER_CONFIG = 0x1A,

        // Interrupt registers
        WAKE_UP_SRC = 0x1B,
        TAP_SRC = 0x1C,
        D6E_SRC = 0x1D,

        // Status data register
        STATUS_REG = 0x1E,

        // Temperature output data register
        OUT_TEMP_L = 0x20,
        OUT_TEMP_H = 0x21,

        // Gyroscope output register
        OUTX_L_G = 0x22,
        OUTX_H_G = 0x23,
        OUTY_L_G = 0x24,
        OUTY_H_G = 0x25,
        OUTZ_L_G = 0x26,
        OUTZ_H_G = 0x27,

        // Accelerometer output registers
        OUTX_L_XL = 0x28,
        OUTX_H_XL = 0x29,
        OUTY_L_XL = 0x2A,
        OUTY_H_XL = 0x2B,
        OUTZ_L_XL = 0x2C,
        OUTZ_H_XL = 0x2D,

        // Sensor hub output registers
        SENSORHUB1_REG = 0x2E,
        SENSORHUB2_REG = 0x2F,
        SENSORHUB3_REG = 0x30,
        SENSORHUB4_REG = 0x31,
        SENSORHUB5_REG = 0x32,
        SENSORHUB6_REG = 0x33,
        SENSORHUB7_REG = 0x34,
        SENSORHUB8_REG = 0x35,
        SENSORHUB9_REG = 0x36,
        SENSORHUB10_REG = 0x37,
        SENSORHUB11_REG = 0x38,
        SENSORHUB12_REG = 0x39,

        // Fifo status registers
        FIFO_STATUS1 = 0x3A,
        FIFO_STATUS2 = 0x3B,
        FIFO_STATUS3 = 0x3C,
        FIFO_STATUS4 = 0x3D,

        // Fifo data output registers
        FIFO_DATA_OUT_L = 0x3E,
        FIFO_DATA_OUT_H = 0x3F,

        // Timestamp output registers
        TIMESTAMP0_REG = 0x40,
        TIMESTAMP1_REG = 0x41,
        TIMESTAMP2_REG = 0x42,

        // Step counter timestamp registers
        STEP_TIMESTAMP_L = 0x49,
        STEP_TIMESTAMP_H = 0x4A,

        // Step counter output registers
        STEP_COUNTER_L = 0x4B,
        STEP_COUNTER_H = 0x4C,

        // Sensor hub output registers
        SENSORHUB13_REG = 0x4D,
        SENSORHUB14_REG = 0x4E,
        SENSORHUB15_REG = 0x4F,
        SENSORHUB16_REG = 0x50,
        SENSORHUB17_REG = 0x51,
        SENSORHUB18_REG = 0x52,

        // Interrupt register
        FUNC_SRC = 0x53,

        // Interrupt registers
        TAP_CFG = 0x58,
        TAP_THS_6D = 0x59,
        INT_DUR2 = 0x5A,
        WAKE_UP_THS = 0x5B,
        WAKE_UP_DUR = 0x5C,
        FREE_FALL = 0x5D,
        MD1_CFG = 0x5E,
        MD2_CFG = 0x5F,
        // External magnetometer raw data output register
        OUT_MAG_RAW_X_L = 0x66,
        OUT_MAG_RAW_X_H = 0x67,
        OUT_MAG_RAW_Y_L = 0x68,
        OUT_MAG_RAW_Y_H = 0x69,
        OUT_MAG_RAW_Z_L = 0x6A,
        OUT_MAG_RAW_Z_H = 0x6B,
    }
    impl Register {
        pub fn addr(self) -> u8 {
            self as u8
        }
    }
}
