
function $id(id) {
    return document.getElementById(id);
}
function $class(id) {
    return document.getElementsByClassName(id);
}
function $name(name) {
    return document.getElementsByName(name);
}
function safelyParseJson(json) {
    // This function cannot be optimised, it's best to
    // keep it small!
    var parsed;
    try {
        parsed = JSON.parse(json);
    } catch (e) {
        parsed = JSON.parse(JSON.stringify(json));
    }
    return parsed // Could be undefined!
}


var websock;
var log_history = [];
function start() {
    var test = "";
    //test = "0:1:1,1:0:1,2:3:0,3:2:0,4:0:0,5:2:0";
    //test = "5;3;0:2:1,1:1:1,2:0:0,3:3:0,4:0:0,5:16:0,6:16:0,7:16:0,8:16:0,9:16:0,10:16:0,11:16:0,12:16:0,13:16:0,14:16:0,15:16:0";
    handset_mix_reset(test);
    $id("logField").scrollTop = $id("logField").scrollHeight;
    websock = new WebSocket('ws://' + window.location.hostname + ':81/');
    websock.onopen = function (evt) { console.log('websock open'); };
    websock.onclose = function(e) {
        console.log('Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        setTimeout(function() {
        start();
        }, 1000);
    };
    websock.onerror = function (evt) { console.log(evt); };
    websock.onmessage = function (evt) {
        //console.log(evt);
        var text = evt.data;
        if (text.startsWith("ELRS_setting_")) {
            var res = text.replace("ELRS_setting_", "");
            res = res.split("=");
            setting_set(res[0], res[1]);
        } else if (text.startsWith("ELRS_tlm_")) {
            var res = text.replace("ELRS_", "");
            res = res.split("=");
            telmetry_set(res[0], res[1]);
        } else if (text.startsWith("ELRS_handset_")) {
            var res = text.replace("ELRS_", "");
            res = res.split("=");
            handset_parse(res[0], res[1]);
        } else {
            var logger = $id("logField");
            var autoscroll = $id("autoscroll").checked;
            var scrollsize = parseInt($id("scrollsize").value, 10);
            while (scrollsize < log_history.length) {
                log_history.shift();
            }
            var date = new Date();
            var n=new Date(date.getTime() - (date.getTimezoneOffset() * 60000)).toISOString();
            log_history.push(n + ' ' + text);
            //logger.value += n + ' ' + text + '\n';
            logger.value = log_history.join('\n');
            if (autoscroll)
                logger.scrollTop = logger.scrollHeight;
        }
    };
}

function saveTextAsFile() {
    var textToWrite = $id('logField').value;
    var textFileAsBlob = new Blob([textToWrite], { type: 'text/plain' });

    var downloadLink = document.createElement("a");
    downloadLink.download = "tx_log.txt";
    downloadLink.innerHTML = "Download File";
    if (window.webkitURL != null) {
        // Chrome allows the link to be clicked without actually adding it to the DOM.
        downloadLink.href = window.webkitURL.createObjectURL(textFileAsBlob);
    } else {
        // Firefox requires the link to be added to the DOM before it can be clicked.
        downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
        downloadLink.onclick = destroyClickedElement;
        downloadLink.style.display = "none";
        document.body.appendChild(downloadLink);
    }

    downloadLink.click();
}

function destroyClickedElement(event) {
    // remove the link from the DOM
    document.body.removeChild(event.target);
}

function setting_set(type, value) {
    var elem = $id(type);
    if (elem) {
        if (type == "region_domain") {
            value = parseInt(value);
            /* Check if handset */
            if (!(value & 0x80)) {
                /* Disable tabs */
                var tabs = $name('handset');
                for (tab in tabs) {
                    tabs[tab].className += " disabled";
                }
            }
            value = value & 0x7F;

            var domain_info = "Regulatory domain ";
            if (value == 0)
                domain_info += "915MHz";
            else if (value == 1)
                domain_info += "868MHz";
            else if (value == 2)
                domain_info += "433MHz";
            else if (value == 3)
                domain_info += "ISM 2400 (BW 0.8MHz)";
            else if (value == 4)
                domain_info += "ISM 2400 (BW 1.6MHz)";
            else
                domain_info += "UNKNOWN";
            elem.innerHTML = domain_info;

            var rf_module = $id("rf_module");
            // update rate options
            var rates = $id("rates_input");
            while (rates.length > 0) {
                rates.remove(rates.length-1);
            }
            var options = [];
            if (3 <= value && value <= 4) {
                options = ['250Hz', '125Hz', '50Hz'];
                if (value == 4) {
                    options.unshift('500Hz');
                }
                rf_module.selectedIndex = 1;
            } else {
                options = ['200Hz', '100Hz', '50Hz'];
                rf_module.selectedIndex = 0;
            }
            for (i = 0; i < options.length; i++) {
                var option = document.createElement("option");
                option.text = options[i];
                option.value = i;
                rates.add(option);
            }
        } else {
            value = value.split(",");
            if (1 < value.length) {
                var max_value = parseInt(value[1], 10);
                if (elem.options[0].value == "R")
                    max_value = max_value + 1; // include reset
                var i;
                // enable all
                for (i = 0; i < elem.length; i++) {
                    elem.options[i].disabled = false;
                }
                // disable unavailable values
                for (i = (elem.length-1); max_value < i; i--) {
                    //elem.remove(i);
                    elem.options[i].disabled = true;
                }
            }
            elem.selectedIndex = [...elem.options].findIndex (option => option.value === value[0]);
        }
    }
}

function setting_send(type, elem=null)
{
    if (elem) {
        websock.send(type + "=" + elem.value);
    } else {
        websock.send(type + "?");
    }
}

function command_stm32(type)
{
    websock.send("stm32_cmd=" + type);
}


/********************* HANDSET *****************************/

function mixer_list_to_dict(value)
{
    var dict = {};
    var i;
    var parts = value.split(";");
    var num_switches = 12, num_aux = 12;
    if (3 == parts.length) {
        num_aux = parseInt(parts[0]);
        num_switches = parseInt(parts[1]);
        value = parts[2];
    }
    dict['aux'] = num_aux;
    dict['total'] = num_aux + 4;
    dict['switch'] = num_switches;
    for (i = 0; i < 16; i++) {
        dict[i] = {'index': '-', 'inv': false};
    }
    value = value.split(",");
    for (var item in value) {
        if (!value[item].length)
            continue
        var mix = value[item].split(":");
        if (3 == mix.length) {
            var idx = parseInt(mix[0]);
            dict[idx].index = mix[1];
            dict[idx].inv = mix[2] == '1' ? true : false;
        }
    }
    return dict;
}

function mixer_add_selection_lst(cell, input="-", values={})
{
    var option;
    var sel = document.createElement("select");
    sel.style.width = "100px";

    /* add options */
    for (var opt in values) {
        option = document.createElement("option");
        option.value = values[opt];
        option.text = opt;
        sel.add(option, values[opt]);
    }
    /* set selected */
    var input_num = parseInt(input);
    if (input != '-' && input_num != undefined) {
        sel.selectedIndex = input_num;
    } else {
        sel.selectedIndex = -1;
    }
    cell.appendChild(sel);
}

function handset_mix_reset(value="")
{
    var iter;
    var table = $id("handset_mixer");
    /* Parse input */
    var mixes = mixer_list_to_dict(value);

    var gimbals = {
        'Gimbal L V': 0, 'Gimbal L H': 1,
        'Gimbal R V': 2, 'Gimbal R H': 3};
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
        var row = table.insertRow();
        var cell = row.insertCell(0);
        if (iter < 4)
            cell.innerHTML = "Analog " + (iter + 1);
        else
            cell.innerHTML = "AUX " + (iter - 3);
        cell = row.insertCell(1);
        cell.style.width = "auto";
        mixer_add_selection_lst(cell, mixes[iter].index,
            ((iter < 4) ? gimbals : switches));

        cell = row.insertCell(2);
        cell.style.width = "100px";
        // creating checkbox element
        var checkbox = document.createElement('input');
        checkbox.type = "checkbox";
        checkbox.name = "invert";
        checkbox.value = iter;
        checkbox.id = "inverted" + iter;
        checkbox.checked = mixes[iter].inv;
        var label = document.createElement('label');
        label.htmlFor = checkbox.id;
        label.appendChild(document.createTextNode('Inverted:'));
        // creating label for checkbox
        cell.appendChild(label);
        cell.appendChild(checkbox);
    }
}

function mixer_send()
{
    var table = $id("handset_mixer");
    var output = "handset_mixer=";
    var index, value;
    for (index = 0; index < table.rows.length; index++) {
        var selected = table.rows[index].cells[1].getElementsByTagName("select")[0];
        if (selected.selectedIndex > -1) {
            /* Channel index */
            output += index.toString(16);
            /* Output channel */
            selected = selected.options[selected.selectedIndex].value;
            output += selected.toString(16);
            /* Inverted */
            var _in = table.rows[index].cells[2].getElementsByTagName("input")[0];
            output += _in.checked ? '1' : '0';
        }
    }
    websock.send(output);
}

/********************* CALIBRATE *****************************/
function handset_calibrate(type)
{
    websock.send("handset_calibrate=" + type);
}

function handset_calibrate_adjust(event)
{
    var msg = "handset_adjust_" + event.target.id + "=";
    var value = parseInt(event.target.value, 10);
    if (value < 0) {
        event.target.value = value = 0;
    } else if (value > 4095) {
        event.target.value = value = 4095;
    }
    if (value < 0x10)
        msg += '00';
    else if (value < 0x100)
        msg += '0'
    msg += value.toString(16);
    websock.send(msg);
}

/********************* TELEMETRY *****************************/

function telmetry_set(type, value)
{
    /* Find correct collection */
    var table = $id(type);
    value = value.split(",");
    for (var item in value) {
        var data = value[item].split(":");
        /* Search row */
        var row = table.rows.namedItem(data[0]);
        if (row) {
            /* update value */
            row.cell[1].innerHTML = data[1];
        }
    }
}

/********************* HANDSET *****************************/
function refresh_values()
{
    websock.send("handset_refresh");
}

function save_values()
{
    websock.send("handset_save");
}

function handset_parse(type, value)
{
    console.log("HANDSET: %o = (%o)", type, value);
    /* Find correct element */
    if (type.includes("_calibrate")) {
        var stat = $id("handset_calibrate_stat");
        stat.innerHTML = 'Calibration state: "' + value + '"';
    } else if (type.includes("_adjust_")) {
        type = type.split("_adjust_")[1];
        value = value.split(":");
        $id(type + '_min').value = value[0];
        $id(type + '_mid').value = value[1];
        $id(type + '_max').value = value[2];
    } else if (type.includes("_mixer")) {
        handset_mix_reset(value);
    }
}