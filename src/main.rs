use esp_idf_hal::{prelude::*, can, gpio, delay};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let mut high_ref = gpio::PinDriver::output(pins.gpio25).unwrap();
    high_ref.set_high().unwrap();

    let filter = can::config::Filter::Standard {filter: 0x7E0, mask: 0xFF0};

    let timing = can::config::Timing::B500K;
    let config = can::config::Config::new().filter(filter).timing(timing);
    let mut can = can::CanDriver::new(peripherals.can, pins.gpio33, pins.gpio32, &config).unwrap();
    can.start().expect("couldn't start can");

    let tx_frame = can::Frame::new(0x7df, can::Flags::None.into(), &[0x02, 0x01, 0x0c /*pid*/, 0x00, 0x00, 0x00, 0x00, 0x00]).unwrap();
    can.transmit(&tx_frame, delay::TickType::new_millis(100).into()).inspect_err(|e| {log::error!("Error transmitting frame: {:?}", e)}).expect("Could not transmit frame");

    if let Ok(rx_frame) = can.receive(delay::TickType::new_millis(100).into()) {
        log::info!("rx {:}:", rx_frame);
        if &rx_frame.data()[0..3] == &[4, 65, 12] {
            log::info!("rpm: {:?}", ((rx_frame.data()[3] as f32) * 256.0 + (rx_frame.data()[4] as f32)) / 4.0)
        }
    } else {
        log::info!("timed out probably");
    }

    log::info!("Hello, world!");
}
