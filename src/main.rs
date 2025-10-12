#![allow(clippy::uninlined_format_args)]

use esp_idf_hal::{delay::FreeRtos, gpio, prelude::*};
use esp_idf_svc::{
    bt::{
        ble::{
            gap::EspBleGap,
            gatt::{server::EspGatts, Permission, Property},
        },
        Ble, BtDriver, BtUuid,
    },
    nvs::EspDefaultNvsPartition,
};
use otgi::{obd, wireless};
use std::sync::Arc;

const SERVICE_UUID: u128 = 0x2cbc6002370f577a928681e04f368400;
const FUEL_USAGE_CHARACTERISTIC_UUID: u128 = 0x56c46fef90390803a71feebcc8650e43;
const RUNCOUNT_CHARACTERISTIC_UUID: u128 = 0xed0cdaa9fc55c2c193a061b6e1f36720;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let nvs_partition = EspDefaultNvsPartition::take().unwrap();
    let bt =
        Arc::new(BtDriver::<Ble>::new(peripherals.modem, Some(nvs_partition.clone())).unwrap());
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
    let runcount: u64 = 4;

    let mut liters_used: f32 = 0.0;

    let fuel_usage_uuid = BtUuid::uuid128(FUEL_USAGE_CHARACTERISTIC_UUID);
    let runcount_uuid = BtUuid::uuid128(RUNCOUNT_CHARACTERISTIC_UUID);
    let ble_server = wireless::Server::new(
        Arc::new(EspBleGap::new(bt.clone()).unwrap()),
        Arc::new(EspGatts::new(bt.clone()).unwrap()),
        wireless::ServerConfiguration {
            services: vec![wireless::ServiceDescriptor {
                uuid: BtUuid::uuid128(SERVICE_UUID),
                is_primary: true,
                characteristics: vec![
                    wireless::CharacteristicDescriptor {
                        uuid: fuel_usage_uuid.clone(),
                        permissions: Permission::Write | Permission::Read,
                        properties: Property::Indicate.into(),
                        max_len: 200,
                        data: liters_used.to_le_bytes().to_vec(),
                    },
                    wireless::CharacteristicDescriptor {
                        uuid: runcount_uuid.clone(),
                        permissions: Permission::Write | Permission::Read,
                        properties: Property::Indicate | Property::Read,
                        max_len: 200,
                        data: runcount.to_le_bytes().to_vec(),
                    },
                ],
            }],
            name: "OTGI",
        },
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

    let mut driver = obd::ObdDriver::try_new(
        peripherals.can,
        pins.gpio33,
        pins.gpio32,
        &Default::default(),
    )
    .unwrap();
    driver.start().expect("Failed to start driver");

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
            log::info!(
                "Estimated fuel usage (L/h): {:?}; {:?} liters used in total",
                usage,
                liters_used
            );
        }

        // UNTESTED
        ble_server
            .indicate(&fuel_usage_uuid, &liters_used.to_le_bytes())
            .unwrap();
    }
}
