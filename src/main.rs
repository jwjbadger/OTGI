use esp_idf_hal::{delay::FreeRtos, gpio, prelude::*};
use otgi::{obd, wireless};
use std::sync::Arc;
use esp_idf_svc::{
    bt::{
        Ble, BtDriver,
        ble::{gap::EspBleGap, gatt::server::EspGatts},
    },
    nvs::{self, EspDefaultNvsPartition},
};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let nvs_partition = EspDefaultNvsPartition::take().unwrap();
    let bt = Arc::new(BtDriver::<Ble>::new(peripherals.modem, Some(nvs_partition.clone())).unwrap());
    let pins = peripherals.pins;

    // Commenting this out until absolutely necessary to reduce the write count to nvs
    /*let nvs_namespace = match nvs::EspNvs::new(nvs_partition, "otgi_data", true) {
        Ok(nvs) => {
            nvs
        }
        Err(e) => panic!("Could't get namespace {e:?}"),
    };

    let runcount = if !nvs_namespace.contains("runcount").unwrap() {
        nvs_namespace.set_u64("runcount", 0).unwrap();
        0
    } else {
        let rc = nvs_namespace.get_u64("runcount").unwrap().unwrap();
        nvs_namespace.set_u64("runcount", rc + 1);
        rc + 1
    };*/
    let runcount = 0;

    let ble_server = wireless::Server::new(
        Arc::new(EspBleGap::new(bt.clone()).unwrap()),
        Arc::new(EspGatts::new(bt.clone()).unwrap()),
    );

    let gap_server = ble_server.clone();
    ble_server
        .gap
        .subscribe(move |event| {
            gap_server.handle_gap_event(event);
        })
        .unwrap();

    let gatts_server = ble_server.clone();
    ble_server
        .gatts
        .subscribe(move |(gatt_intf, event)| {
            gatts_server.handle_gatts_event(gatt_intf, event);
        })
        .unwrap();

    ble_server.gatts.register_app(wireless::APP_ID).unwrap();

    log::info!("BLE Gap and Gatts initialized");

    let mut high_ref = gpio::PinDriver::output(pins.gpio25).unwrap();
    high_ref.set_high().unwrap();

    let mut driver = obd::ObdDriver::try_new(peripherals.can, pins.gpio33, pins.gpio32, &Default::default()).unwrap();
    driver.start().expect("Failed to start driver");

    let mut liters_used: f32 = 0.0;
    loop {
        // TODO: See if it's possible to reduce delay
        let maf = driver.query(obd::PID::MassAirFlow);
        FreeRtos::delay_ms(100);
        let stft = driver.query(obd::PID::ShortTermFuelTrimBankOne);
        FreeRtos::delay_ms(100);
        let ltft = driver.query(obd::PID::LongTermFuelTrimBankOne);
        FreeRtos::delay_ms(100);

        if let (Ok(maf), Ok(stft), Ok(ltft)) = (maf, stft, ltft) {
            let usage = (maf * 3600.0) / ((14.7 * (1.0 + ((stft + ltft) / 100.0))) * 740.0);
            liters_used += (usage / 3600.0) * 0.3; // rough approximation (300 ms)
            log::info!("Estimated fuel usage (L/h): {:?}; {:?} liters used in total", usage, liters_used);
        }

        // UNTESTED
        ble_server.indicate(&liters_used.to_le_bytes()).unwrap();
    }
}
