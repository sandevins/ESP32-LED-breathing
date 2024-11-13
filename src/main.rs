use anyhow::Result;
use esp_idf_svc::hal::{delay::FreeRtos, peripherals::Peripherals};
use core::time::Duration;
use rgb::RGB8;
use esp_idf_svc::hal::{
    gpio::OutputPin,
    peripheral::Peripheral,
    rmt::{config::TransmitConfig, FixedLengthSignal, PinState, Pulse, RmtChannel, TxRmtDriver},
};

pub struct WS2812RMT<'a> {
    tx_rtm_driver: TxRmtDriver<'a>,
}

impl<'d> WS2812RMT<'d> {
    // Rust ESP Board gpio2,  ESP32-C3-DevKitC-02 gpio8
    pub fn new(
        led: impl Peripheral<P = impl OutputPin> + 'd,
        channel: impl Peripheral<P = impl RmtChannel> + 'd,
    ) -> Result<Self> {
        let config = TransmitConfig::new().clock_divider(2);
        let tx = TxRmtDriver::new(channel, led, &config)?;
        Ok(Self { tx_rtm_driver: tx })
    }
    pub fn set_pixel(&mut self, rgb: RGB8) -> Result<()> {
        let color: u32 = ((rgb.g as u32) << 16) | ((rgb.r as u32) << 8) | rgb.b as u32;
        let ticks_hz = self.tx_rtm_driver.counter_clock()?;
        let t0h = Pulse::new_with_duration(ticks_hz, PinState::High, &ns(350))?;
        let t0l = Pulse::new_with_duration(ticks_hz, PinState::Low, &ns(800))?;
        let t1h = Pulse::new_with_duration(ticks_hz, PinState::High, &ns(700))?;
        let t1l = Pulse::new_with_duration(ticks_hz, PinState::Low, &ns(600))?;
        let mut signal = FixedLengthSignal::<24>::new();
        for i in (0..24).rev() {
            let p = 2_u32.pow(i);
            let bit = p & color != 0;
            let (high_pulse, low_pulse) = if bit { (t1h, t1l) } else { (t0h, t0l) };
            signal.set(23 - i as usize, &(high_pulse, low_pulse))?;
        }
        self.tx_rtm_driver.start_blocking(&signal)?;

        Ok(())
    }
}
fn ns(nanos: u64) -> Duration {
    Duration::from_nanos(nanos)
}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    // Onboard RGB LED pin
    // Rust ESP Board gpio2,  ESP32-C3-DevKitC-02 gpio8
    let led = peripherals.pins.gpio8;
    let channel = peripherals.rmt.channel0;
    let mut ws2812 = WS2812RMT::new(led, channel)?;
    loop {
        // Smooth transition from Red -> Green
        for g in 0..=100 {
            ws2812.set_pixel(rgb::RGB8::new(100 - g, g, 0))?;
            FreeRtos::delay_ms(5);
        }

        // Smooth transition from Green -> Blue
        for r in (0..=100).rev() {
            ws2812.set_pixel(rgb::RGB8::new(0, r, 100 - r))?;
            FreeRtos::delay_ms(5);
        }

        // Smooth transition from Blue -> Red
        for b in 0..=100 {
            ws2812.set_pixel(rgb::RGB8::new(b, 0, 100 - b))?;
            FreeRtos::delay_ms(5);
        }
    }
}
