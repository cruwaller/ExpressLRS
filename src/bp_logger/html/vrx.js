import * as common from "./common.js";
export let cmn = common;

// HDZero supported bands and channels
const hdzero_channelFreqTable = {
  "E": [5705,   -1,   -1,   -1,   -1,   -1,   -1,   -1], // E
  "F": [5740, 5760,   -1, 5800,   -1,   -1,   -1,   -1], // F / Airwave
  "R": [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], // R / Immersion Raceband
};

const fusion_channelFreqTable = {
  "A": [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725], // A
  "B": [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866], // B
  "E": [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945], // E
  "F": [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880], // F / Airwave
  "R": [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], // R / Immersion Raceband
  "L": [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621], // L
};

export function start() {
  const fea_debug = {
    "recording":1, "osd_text":1, "laptimer":1, "espnow":1,
    "telemetrydebug":1,
    "osd_row": 1, "osd_row_max": 88, "osd_col": 2, "osd_col_max": 99
  };
  feature_config(fea_debug);

  common.common_init();

  // configure events
  const events = {
    "fea_config": function(e) {
      const config = JSON.parse(e.data);
      feature_config(config);
      common.msp_vtx_freq(config.vtxfreq);},
    "vrx_version": function(e) {
      $id("firmware_version_vrx").innerHTML = e.data;},
    "vtxfreq": function(e) {
      const config = JSON.parse(e.data);
      common.msp_vtx_freq(config.vtxfreq);},
  };
  common.events_init(events);

  common.websocket_init(handle_input_msg);
}

function handle_input_msg(msgid, payload) {
  switch (msgid) {
    case common.WSMSGID_RECORDING_CTRL:
      recording_control_received(!!payload.getUint8());
      break;
    default:
        return false;
  }
  return true;
}

/********************* UI CONFIG *************************/
function feature_config(config) {
  if (config.model == "HDZero") {common.vtx_table_set(hdzero_channelFreqTable);}
  else if (config.model == "TBS Fusion") {common.vtx_table_set(fusion_channelFreqTable);}

  $id("recording_control").style.display = !!config.recording ? "block" : "none";
  $id("osd_text_control").style.display = !!config.osd_text ? "block" : "none";
  $id("laptimer_control").style.display = !!config.laptimer ? "block" : "none";
  $id("espnow_control").style.display = !!config.espnow ? "block" : "none";
  $id("vrxversion_control").style.display = !!config.vrxversion ? "block" : "none";
  $id("telemetry_debug").style.display = !!config.telemetrydebug ? "block" : "none";

  if (config.osd_row_max == undefined || config.osd_col_max == undefined) {
    $id("laptimer_osd_pos_control").style.display = "none";
  } else {
    const _row_obj = $id("laptimer_osd_row");
    const _col_obj = $id("laptimer_osd_col");
    _row_obj.value = config.osd_row;
    _row_obj.max = config.osd_row_max;
    _col_obj.value = config.osd_col;
    _col_obj.max = config.osd_col_max;
  }
  $id("laptimer_osd_timeout").value = config.osd_timeout;
}

/********************* RECORDING *************************/
export function osd_pos_send(event) {
  const _row_obj = $id("laptimer_osd_row");
  const _col_obj = $id("laptimer_osd_col");
  var row = parseInt(_row_obj.value, 10);
  if (_row_obj.max < row) {row = _row_obj.value = _row_obj.max};
  var col = parseInt(_col_obj.value, 10);
  if (_col_obj.max < col) {col = _col_obj.value = _col_obj.max};
  const payload = [row & 0xff, (row >> 8) & 0xff, col & 0xff, (col >> 8) & 0xff];
  common.message_send_binary(common.WSMSGID_LAPTIMER_OSD_POS, payload);
}

export function osd_timeout_send(event) {
  const _to_obj = $id("laptimer_osd_timeout");
  var timeout = parseInt(_to_obj.value, 10);
  if (_to_obj.max < timeout) {timeout = _to_obj.value = _to_obj.max};
  const payload = [timeout & 0xff, (timeout >> 8) & 0xff];
  common.message_send_binary(common.WSMSGID_LAPTIMER_OSD_TIMEOUT, payload);
}

/********************* RECORDING *************************/
export function recording_control_send(btn) {
  var start;
  if (btn.innerHTML == "START") {
    btn.innerHTML = "STOP";
    start = true;
  } else {
    btn.innerHTML = "START";
    start = false;
  }
  common.message_send_binary(common.WSMSGID_RECORDING_CTRL, [start]);
}

function recording_control_received(state) {
  const button = $id("recording_state");
  $class_del(button, "green"); $class_del(button, "red");
  $class_add(button, state ? "red" : "green");
  button.innerHTML = state ? "STOP" : "START";
}
/********************* DEBUG *************************/
export function message_send_tlm_debug(field, data) {
  var msg_id = undefined;
  const hexToDecimal = hex => parseInt(hex, 16);
  data = data.split(",");
  data = data.map(t => hexToDecimal(t));
  if (field == "battery") {
    console.log("BATT:", data);
    msg_id = common.WSMSGID_CRSF_DBG_TLM_BATT;
    if (data.length < 8) {
      console.error("Batt data is too short");
    }
  } else if (field == "link") {
    console.log("LINK:", data);
    msg_id = common.WSMSGID_CRSF_DBG_TLM_LINK;
    if (data.length < 10) {
      console.error("Link data is too short");
    }
  } else if (field == "gps") {
    console.log("GPS:", data);
    msg_id = common.WSMSGID_CRSF_DBG_TLM_GPS;
    if (data.length < 16) {
      console.error("GPS data is too short");
    }
  } else if (field == "bytes") {
    console.log("BYTES:", data);
    msg_id = common.WSMSGID_CRSF_DBG_TLM_BYTES;
  } else {
    console.error("Invalid type: ", field);
    return;
  }
  var sendarray = new ArrayBuffer(2 + data.length);
  var view = new Uint8Array(sendarray);
  view[0] = msg_id & 0xFF;
  view[1] = (msg_id >> 8) & 0xFF;
  for (var iter = 0; iter < data.length; iter++)
      view[2+iter] = data[iter];
  return websock_validate_and_send(sendarray);
}

export function message_send_lap_time() {
  const _lap = $id("dbg_lap_idx").value;
  const _time = $id("dbg_lap_time").valueAsDate;
  if (!_time || !_lap) return;

  const d = new Date(_time);
  var ms = 0;
  ms += d.getMilliseconds();
  ms += d.getSeconds() * 1000;
  ms += d.getMinutes() * 60 * 1000;

  console.log("Lap %o : %o ==> %o ms", _lap, _time, ms);

  var data = [_lap, (ms & 0xff), ((ms >> 8) & 0xff), ((ms >> 16) & 0xff), ((ms >> 24) & 0xff)];

  const msg_id = common.WSMSGID_CRSF_DBG_LAPTIME;
  var sendarray = new ArrayBuffer(2 + data.length);
  var view = new Uint8Array(sendarray);
  view[0] = msg_id & 0xFF;
  view[1] = (msg_id >> 8) & 0xFF;
  for (var iter = 0; iter < data.length; iter++)
      view[2+iter] = data[iter];
  console.log("sendarray: %o", view);
  return websock_validate_and_send(sendarray);
}
