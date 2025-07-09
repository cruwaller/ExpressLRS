import * as common from "./common.js";
export let cmn = common;

const ws_msgid_lookup = {
    // STM32 slave control
    "stm32_reset":       common.WSMSGID_STM32_RESET,

    "refresh":           common.WSMSGID_ELRS_SETTINGS,
    "rf_module":         common.WSMSGID_ELRS_DOMAIN,
    "rate":              common.WSMSGID_ELRS_RATE,
    "power":             common.WSMSGID_ELRS_POWER,
    "telemetry":         common.WSMSGID_ELRS_TLM,
    "rf_pwr":            common.WSMSGID_ELRS_RF_POWER_TEST,

    "handset_reload":    common.WSMSGID_HANDSET_RELOAD,
    "handset_save":      common.WSMSGID_HANDSET_SAVE,
};

const elrs_settings_lookup = {
    0: {"rfidx": 1, "rates": ['200Hz', '100Hz', '50Hz'], "info": "915MHz"},
    1: {"rfidx": 1, "rates": ['200Hz', '100Hz', '50Hz'], "info": "868MHz"},
    2: {"rfidx": 1, "rates": ['200Hz', '100Hz', '50Hz'], "info": "433MHz"},
    3: {"rfidx": 2, "rates": ['500Hz', '250Hz', '125Hz', '50Hz'], "info": "2400MHz ISM (LoRa)"},
    4: {"rfidx": 2, "rates": ['500Hz', '250Hz', '125Hz', '50Hz'], "info": "2400MHz ISM (LoRa)"},
    5: {"rfidx": 3, "rates": ['1000Hz', '500Hz', '250Hz'], "info": "2400MHz ISM (FLRC)"},
    6: {"rfidx": 4, "rates": ['DVDA500Hz', 'DVDA250Hz', 'LORA500Hz', 'LORA250Hz'], "info": "2400MHz ISM (VANILLA 3.x)"},
    7: {"rfidx": 5, "rates": ['DVDA500Hz', 'DVDA250Hz', 'LORA500Hz', 'LORA250Hz'], "info": "2400MHz ISM (VANILLA 4.x)"},
};

export function start() {
    const fea_debug = {"laptimer":1, "espnow":1, "handset":1};
    feature_config(fea_debug);

    common.common_init();

    // configure events
    const events = {
        "fea_config": function(e) {
            const config = JSON.parse(e.data);
            feature_config(config);},
        "elrs_version": function(e) {
            $id("firmware_version_elrs").innerHTML = e.data;},
        "elrs_settings": function(e) {
            const settings = JSON.parse(e.data);
            handle_settings(settings);
            handset_mixer(settings['mixer']);
            handset_calibrate_adjust(settings['gimbals']);},
    };
    common.events_init(events);

    common.websocket_init(handle_input_msg);
}

function handle_input_msg(msgid, payload) {
    switch (msgid) {
        case common.WSMSGID_ELRS_SETTINGS:
            handle_settings(settings_parse(payload));
            break;
        case common.WSMSGID_HANDSET_CALIBRATE:
            if (calibrate_btn != null) {
                calibrate_btn.disabled = false;
                calibrate_btn = null;
                $id("handset_calibrate_stat").innerHTML = 'Calibration failed!';
            }
            break;
        case common.WSMSGID_HANDSET_BATT_INFO:
            handset_battery_value(payload);
            break;
        case common.WSMSGID_HANDSET_ADJUST:
            handset_calibrate_adjust(handset_calibrate_adjust_to_dict(payload));
            break;
        case common.WSMSGID_HANDSET_MIXER:
            handset_mixer(mixer_list_to_dict(payload));
            break;
        case common.WSMSGID_HANDSET_TLM_LINK_STATS:
            handset_telemetry_link_stats(payload);
            break;
        case common.WSMSGID_HANDSET_TLM_BATTERY:
            handset_telemetry_battery(payload);
            break;
        case common.WSMSGID_HANDSET_TLM_GPS:
            handset_telemetry_gps(payload);
            break;
        default:
            return false;
    }
    return true;
}

/********************* UI CONFIG *************************/
function feature_config(config) {
    $id("espnow_control").style.display = !!config.espnow ? "block" : "none";
    $id("laptimer_control").style.display = !!config.laptimer ? "block" : "none";
    $id("laptimer_aux_panel").style.display = !!config.laptimer ? "block" : "none";
    $id("recording_aux_panel").style.display = !!config.recording_ctrl ? "block" : "none";
    /* Hide or show handset specific tabs */
    const isHandset = !!config.handset ? "block" : "none";
    var tabs = $name('handset');
    for (var tab of tabs) {
        tab.style.display = isHandset;
    }
}

/******************* WEB SOCKET *************************/
export function message_send(elem=null, bytesize=1)
{
    var sendarray = new ArrayBuffer(2 + ((elem == null) ? 0 : bytesize));  // MSGID + value
    var view = new Uint8Array(sendarray);

    if (elem) {
        const msg_id = ws_msgid_lookup[elem.name];
        view[0] = msg_id & 0xFF;
        view[1] = (msg_id >> 8) & 0xFF;
        for (var iter = 0; iter < bytesize; iter++)
            view[2+iter] = (elem.value >> (8*iter)) & 0xFF;
    } else {
        view[0] = common.WSMSGID_ELRS_SETTINGS & 0xFF;
        view[1] = (common.WSMSGID_ELRS_SETTINGS >> 8) & 0xFF;
    }
    common.websock_validate_and_send(sendarray);
}

/***************** ELRS SETTINGS *********************/
export function rf_module_changed(elem)
{
    $id("rf_module").disabled = true;
    $id("rates_input").disabled = true;
    $id("power_input").disabled = true;
    $id("tlm_input").disabled = true;
    message_send(elem);
}

function handle_setting_region(domain)
{
    if (domain == undefined) return;
    const FLAGS = {
        dual: 0x80,
        handset: 0x40,
        flrc: 0x20,
        mask: 0x1F,
    };

    const rf_module = $id("rf_module");
    if (!(domain & FLAGS.dual)) {
        /* Disable RF selection if not dual module */
        rf_module.disabled = true;
    } else {
        rf_module.disabled = false;
    }

    /*if (!(domain & FLAGS.handset)) {
        // Disable tabs if not handset
        var tabs = $name('handset');
        for (var tab of tabs) {
            tab.className += " disabled";
        }
    }*/
    domain = domain & FLAGS.mask;

    const domain_params = elrs_settings_lookup[domain];
    if (domain_params == undefined) {
        rf_module.disabled = true;
        $id("rates_input").disabled = true;
        $id("power_input").disabled = true;
        $id("tlm_input").disabled = true;
        $id("region_domain").innerHTML = "Invalid domain: " + domain;
        return false;
    }

    // Store domain index for later use
    rf_module.domain_in_use = domain;
    rf_module.selectedIndex = domain_params.rfidx;
    $id("region_domain").innerHTML = "Domain " + domain_params.info;

    // update rate options
    const rates = $id("rates_input");
    while (rates.length > 0) {
        rates.remove(rates.length-1);
    }
    const options = domain_params.rates;
    for (var i = 0; i < options.length; i++) {
        var option = document.createElement("option");
        option.text = options[i];
        option.value = i;
        rates.add(option);
    }
    return true;
}

function handle_setting_generic(elem, value, max_value=null)
{
    if (value == undefined) return;
    if (max_value != null) {
        max_value = max_value + 2; // include reset and dummy
        // enable all
        for (var i = 0; i < elem.length; i++)
            elem.options[i].disabled = false;
        // disable unavailable values
        for (var i = (elem.length-1); max_value < i; i--)
            elem.options[i].disabled = true;
    }
    value = value.toString();
    elem.selectedIndex = [...elem.options].findIndex (option => option.value === value);
    elem.disabled = false;
}

function handle_settings(settings)
{
    console.log("ELRS settings: ", settings);
    if (handle_setting_region(settings.region)) {
        handle_setting_generic($id("rates_input"), settings.rate);
        handle_setting_generic($id("power_input"), settings.power, settings.power_max);
        handle_setting_generic($id("tlm_input"), settings.telemetry);
        // disable telemetry options if vanilla mode
        $id("tlm_input").disabled = ($id("rf_module").domain_in_use == 6);
    }
    common.msp_vtx_freq(settings.vtxfreq);
    $id("laptimer_aux").value = !!settings.laptimer_aux ? settings.laptimer_aux : 255;
    $id("recording_aux").value = !!settings.recording_aux ? settings.recording_aux : 255;
}

function settings_parse(payload)
{
    const settings = {
        "region": payload.nextUint8(),
        "rate": payload.nextUint8(),
        "power": payload.nextUint8(),
        "power_max": payload.nextUint8(),
        "telemetry": payload.nextUint8(),
        "vtxfreq": (payload.nextUint8() << 8) + payload.nextUint8(),
    };
    return settings;
}

/********************* STM32 *******************************/

export function command_stm32(type)
{
    var elem = Object();
    elem.name = "stm32_" + type;
    message_send(elem, 0);
}


/********************* HANDSET MIXER *********************/

function mixer_list_to_dict(payload)
{
    var mixer_config = {};
    mixer_config['aux'] = payload.nextUint8();
    mixer_config['total'] = mixer_config['aux'] + 4;
    mixer_config['switch'] = payload.nextUint8();
    const config = [];
    for (var i = 0; i < 16; i++) {
        config.push({'index': -1, 'inv': false, 'scale': 1.0});
    }
    for (var i = 0; i < 16 && i < ((payload.byteLength - 2) / 4); i++) {
        const idx = payload.nextUint8();
        config[idx].index = payload.nextUint8();
        config[idx].inv = payload.nextUint8();
        config[idx].scale = payload.nextUint8();
    }
    mixer_config['config'] = config;
    return mixer_config;
}

function mixer_add_selection_lst(cell, input=-1, values={})
{
    var option;
    var sel = document.createElement("select");
    sel.style.width = "100px";

    option = document.createElement("option");
    option.value = -1;
    option.text = "";
    sel.add(option, option.value);

    /* add options */
    for (const opt in values) {
        option = document.createElement("option");
        option.value = values[opt];
        option.text = opt;
        sel.add(option, values[opt]);
    }
    /* set selected */
    sel.selectedIndex = input;
    cell.appendChild(sel);
}

function handset_mixer(mixes)
{
    if (mixes == undefined) return;
    var iter;
    var table = $id("handset_mixer");

    var gimbals = {
        'Gimbal L1': 0, 'Gimbal L2': 1,
        'Gimbal R1': 2, 'Gimbal R2': 3};
    var switches = {};
    for (iter = 0; iter < mixes['switch']; iter++) {
        switches['Switch ' + (iter+1)] = iter;
    }

    /* clean table */
    while (table.rows.length) {
        table.deleteRow(table.rows.length-1);
    }
    /* write values */
    for (iter = 0; iter < mixes['total']; iter++) {
        const mix_cfg = mixes['config'][iter];
        var row = table.insertRow();
        var cell = row.insertCell(0);
        if (iter < 4)
            cell.innerHTML = "Analog " + (iter + 1);
        else
            cell.innerHTML = "AUX " + (iter - 3);
        cell = row.insertCell(1);
        cell.style.width = "auto";
        mixer_add_selection_lst(cell, mix_cfg.index,
            ((iter < 4) ? gimbals : switches));

        cell = row.insertCell(2);
        cell.style.width = "100px";
        // creating checkbox element
        var checkbox = document.createElement('input');
        checkbox.type = "checkbox";
        checkbox.name = "invert";
        checkbox.value = iter;
        checkbox.id = "inverted" + iter;
        checkbox.checked = mix_cfg.inv ? true : false;
        var label = document.createElement('label');
        label.htmlFor = checkbox.id;
        label.appendChild(document.createTextNode('Inverted:'));
        // creating label for checkbox
        cell.appendChild(label);
        cell.appendChild(checkbox);

        cell = row.insertCell(3);
        cell.style.width = "110px";
        if (iter < 4) {
            var scale = document.createElement("input");
            scale.type = "number";
            scale.name = "scale";
            scale.min = 0.1;
            scale.max = 1.0;
            scale.step = 0.05;
            scale.value = (mix_cfg.scale) ? (mix_cfg.scale.toFixed(4) / 100.) : 1.0;
            scale.id = "scale" + iter;
            scale.style.width = "50px";
            // creating label for checkbox
            label = document.createElement('label');
            label.htmlFor = scale.id;
            label.style.width = "40px";
            label.appendChild(document.createTextNode('Scale:'));
            cell.appendChild(label);
            cell.appendChild(scale);
        }
    }
}

export function handset_mixer_send()
{
    var table = $id("handset_mixer");
    var output = [];
    for (var index = 0; index < table.rows.length; index++) {
        var rows = table.rows[index];
        var selected = rows.cells[1].getElementsByTagName("select")[0];
        if (-1 < selected.selectedIndex) {
            selected = selected.options[selected.selectedIndex].value;
            if (selected < 0) continue;
            /* Channel index */
            output.push(index);
            /* Output channel */
            output.push(selected);
            /* Inverted */
            var _in = rows.cells[2].getElementsByTagName("input")[0];
            output.push(_in.checked ? 1 : 0);

            if (index < 4) {
                /* add scale */
                _in = rows.cells[3].getElementsByTagName("input")[0];
                _in = parseFloat(_in.value) * 100;
                if (100 <= _in)
                    _in = 0;
                else if (_in <= 10)
                    _in = 10;
                output.push(_in);
            }
        }
    }
    common.message_send_binary(common.WSMSGID_HANDSET_MIXER, output);
}

/********************* CALIBRATE *****************************/
const mapping_calibrate = {
    "L1_min": 0x01, "L1_mid": 0x02, "L1_max": 0x03,
    "L2_min": 0x11, "L2_mid": 0x12, "L2_max": 0x13,
    "R1_min": 0x21, "R1_mid": 0x22, "R1_max": 0x23,
    "R2_min": 0x31, "R2_mid": 0x32, "R2_max": 0x33,
};
var calibrate_btn = null;
export function handset_calibrate_auto_send(btn, type)
{
    if (calibrate_btn != null) {
        return;
    }
    btn.disabled = true;
    calibrate_btn = btn;
    common.message_send_binary(common.WSMSGID_HANDSET_CALIBRATE, [mapping_calibrate[type]]);
}

export function handset_calibrate_adjust_send(event)
{
    var value = parseInt(event.target.value, 10);
    if (value < 0) {
        event.target.value = value = 0;
    } else if (value > 4095) {
        event.target.value = value = 4095;
    }
    const data = [
        mapping_calibrate[event.target.id],
        (value & 0xFF),
        ((value >> 8) & 0xFF)];
    common.message_send_binary(common.WSMSGID_HANDSET_ADJUST, data);
}

function handset_calibrate_adjust_to_dict(payload)
{
    const config = [];
    for (var iter = 0; iter < 4; iter++) {
        const offset = iter * 6; // 3 * 2B
        config[iter] = {
            'min': payload.getUint16(offset+0, true),
            'mid': payload.getUint16(offset+2, true),
            'max': payload.getUint16(offset+4, true),
        };
    }
}

function handset_calibrate_adjust(calibration)
{
    if (calibration == undefined) return;
    //const calib_table = $id("calibrate");
    //for (var row of calib_table.rows)
    //    console.log(row);
    const mapping = {
        0 : 'L1_', 1 : 'L2_',
        2 : 'R1_', 3 : 'R2_',
    };
    for (const iter in calibration) {
        const type = mapping[iter];
        const values = calibration[iter];
        $id(type + 'min').value = values['min'];
        $id(type + 'mid').value = values['mid'];
        $id(type + 'max').value = values['max'];
    }
    if (calibrate_btn != null) {
        calibrate_btn.disabled = false;
        calibrate_btn = null;
        $id("handset_calibrate_stat").innerHTML = 'Calibration ready!';
    }
}

/******************** BATTERY MONITORING *********************/
function handset_battery_value(payload)
{
    $id("battery_voltage").innerHTML =
        (payload.getUint32(0) / 1000.).toFixed(1) + "V";
    $id("battery_scale").value = payload.getUint8(4);
    $id("battery_warning").value = payload.getUint8(5);
}

export function handset_battery_adjust() {
    const data = [
        parseInt($id("battery_scale").value, 10),
        parseInt($id("battery_warning").value, 10),
    ];
    common.message_send_binary(common.WSMSGID_HANDSET_BATT_CONFIG, data);
}

export function handset_laptimer_aux_set(aux) {
    const data = [parseInt(aux, 10)];
    common.message_send_binary(common.WSMSGID_HANDSET_LAPTIMER_AUX, data);
}

export function handset_recording_aux_set(aux) {
    const data = [parseInt(aux, 10)];
    common.message_send_binary(common.WSMSGID_HANDSET_RECORDING_AUX, data);
}

/********************* TELEMETRY *****************************/
var gps_mode = "kmh";
function convert_gps_speed(speed)
{
    speed = parseInt(speed, 10);
    if (gps_mode == "kmh") {
        speed = ((speed) * 36 / 1000);
        speed = speed.toString() + " km/h";
    } else {
        speed = ((speed) * 10000 / 5080 / 88);
        speed = speed.toString() + " mp/h";
    }
    return speed;
}

function convert_gps_value(val)
{
    val = parseInt(val, 10).toFixed(2);
    const GPS_DEGREES_DIVIDER = 10000000.;
    const result = (val / GPS_DEGREES_DIVIDER).toString();
    return result;
}

function handset_telemetry_link_stats(payload)
{
    $id("tlm_ul_RSSI1").innerHTML = payload.nextInt8().toString() + " dBm";
    $id("tlm_ul_RSSI2").innerHTML = payload.nextInt8().toString() + " dBm";
    $id("tlm_ul_LQ").innerHTML = payload.nextUint8();
    $id("tlm_ul_SNR").innerHTML = (payload.nextInt8().toFixed(1) / 10.).toString() + " dB";
    //payload.nextUint8(); // active_antenna
    //payload.nextUint8(); // rf_Mode
    //payload.nextUint8(); // uplink_TX_Power
    payload.offset_next += 3; // skip unused values
    $id("tlm_dl_RSSI1").innerHTML = payload.nextInt8().toString() + " dBm";
    $id("tlm_dl_LQ").innerHTML = payload.nextUint8();
    $id("tlm_dl_SNR").innerHTML = (payload.nextInt8().toFixed(1) / 10.).toString() + " dB";
    const now = time_current();
    $id("tlm_ul_updated").innerHTML = now;
    $id("tlm_dl_updated").innerHTML = now;

    handset_telemetry_battery(payload, now);
}

function handset_telemetry_battery(payload, now=null)
{
    $id("tlm_batt_V").innerHTML = (payload.nextUint16().toFixed(1) / 10.).toString() + " V";
    $id("tlm_batt_A").innerHTML = (payload.nextUint16().toFixed(1) / 10.).toString() + " A";
    const capacity = (payload.nextUint8() << 16) + (payload.nextUint8() << 8) + payload.nextUint8();
    $id("tlm_batt_C").innerHTML = capacity.toString() + " mAh";
    $id("tlm_batt_R").innerHTML = payload.nextUint8().toString() + " %";
    if (now == null) now = time_current();
    $id("tlm_batt_updated").innerHTML = now;
}

function handset_telemetry_gps(payload)
{
    $id("tlm_gps_lat").innerHTML = convert_gps_value(payload.nextInt32(true));
    $id("tlm_gps_lon").innerHTML = convert_gps_value(payload.nextInt32(true));
    $id("tlm_gps_speed").innerHTML = convert_gps_speed(payload.nextUint16(true));
    $id("tlm_gps_head").innerHTML = (payload.nextUint16(true).toFixed(1) / 10.).toString() + " deg";
    $id("tlm_gps_alt").innerHTML = (payload.nextInt16(true) - 1000).toString() + " m";
    $id("tlm_gps_sat").innerHTML = payload.nextUint8();
    $id("tlm_gps_updated").innerHTML = time_current();
}
