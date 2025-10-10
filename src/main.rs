use esp_idf_hal::{delay::FreeRtos, gpio, prelude::*};
use otgi::obd;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let mut high_ref = gpio::PinDriver::output(pins.gpio25).unwrap();
    high_ref.set_high().unwrap();

    let mut driver = obd::ObdDriver::try_new(peripherals.can, pins.gpio33, pins.gpio32, &Default::default()).unwrap();
    driver.start().expect("Failed to start driver");

    loop {
        log::info!("Run Time: {:?}", driver.query(obd::PID::RunTime));
        FreeRtos::delay_ms(1000);
        log::info!("rpm: {:?}", driver.query(obd::PID::EngineSpeed));
        FreeRtos::delay_ms(1000);
        log::info!("Speed: {:?}", driver.query(obd::PID::VehicleSpeed));
        FreeRtos::delay_ms(1000);
        log::info!("Throttle Position: {:?}", driver.query(obd::PID::ThrottlePosition));
        FreeRtos::delay_ms(1000);*/
    }
}
