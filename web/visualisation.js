function documentReady(fn) {
    // see if DOM is already available
    if (document.readyState === "complete" || document.readyState === "interactive") {
        // call on next available tick
        setTimeout(fn, 100);
    } else {
        document.addEventListener("DOMContentLoaded", fn);
    }
}

function add_label(grid, channel_name, color, box=null, button=null) {
    div = document.createElement("div");
    div.className = "label";
    div.id = channel_name;
    div.style = "background-color: " + color;
    var p = document.createElement("div");
    p.className = "channel_name";
    p.id = channel_name;
    p.textContent = channel_name;
    // tn = document.createTextNode(channel_name);

    if (box != null) {
        div.appendChild(box);
    }
    // div.appendChild(tn);
    div.appendChild(p);
    if (button != null) {
          div.appendChild(button);
    }
    grid.appendChild(div);
    return div;
}


documentReady(function() {

    window.addEventListener("gamepadconnected", (event) => {
        console.log("A gamepad connected:");
        console.log(event.gamepad);
    });

      window.addEventListener("gamepaddisconnected", (event) => {
        console.log("A gamepad disconnected:");
        console.log(event.gamepad);
    });

    ws = start_websocket(5678);
    graphs = new GraphCollection();
    //
    // graph_analog = new Graph('canvas_analog');
    // graph_analog.msPerPixel = 5;

    graph_states = new Graph('canvas_states');
    graph_states.msPerPixel = 5;
    //
    // graph_digital = new Graph('canvas_digital');
    // graph_digital.msPerPixel = 5;
    //
    // var unit_fields_analog = [];
    // var unit_names_analog = ['V', 'V', 'V', 'V', 'V', 'V', 'V', 'V'];
    // for (let i=0; i<8; i++) {
    //     lbl = document.createElement('div');
    //     lbl.className = 'unit_field analog';
    //     lbl.id = "analog_" + i.toString();
    //     unit_fields_analog.push(lbl);
    // }

    var unit_fields_states = [];
    var unit_names_states = ['count', 'cm', 'cm', 'cm/s', 'cm/s²', 'mA', 'None', 'None'];
    var field_names_states = ['encoder', 'target_pos', 'current_pos', 'speed', 'acceleration', 'RMS', 'None', 'None'];
    for (let i=0; i<8; i++) {
        lbl = document.createElement('div');
        lbl.className = 'unit_field states';
        lbl.id = "states_" + i.toString();
        unit_fields_states.push(lbl);
    }

    // var indicators = [];
    // for (i=0; i<24; i++) {
    //     ind = document.createElement('div');
    //     ind.className = "indicator";
    //     ind.id = "digital"+i.toString();
    //     indicators.push(ind);
    // }
    //
    // var buttons = [];
    // for (i=0; i<8; i++) {
    //     btn = document.createElement('button');
    //     var text = document.createTextNode('toggle');
    //     btn.appendChild(text)
    //     btn.className = "toggle_button";
    //     btn.id = "digital"+(16+i).toString();
    //     btn.onclick = btn_clicked;
    //     buttons.push(btn);
    // }

    // // analog graph
    // var label_grid = document.querySelector('#analog_grid');
    // var num_lines = 8;
    // for (let i = 0; i < num_lines; i++) {
    //     const line = graph_analog.add_line();
    //     line.scaleFactor = 1/2**16;
    //     line.offset = 0;
    //     line.color = 'hsl(' + (180/num_lines*i).toString() + ', 80%, 60%)';
    //     add_label(label_grid, 'analog_input_' + i.toString(), line.color, button=unit_fields_analog[i]);
    // }

    // states graph
    label_grid = document.querySelector('#states_grid');
    num_lines = 8;
    var scale_factors = [1/2**6, 1/2**15, 1/2**15, 1/2**16,
                        1/25, 1/2**22, 1/100, 1/100];
    for (let i = 0; i < num_lines; i++) {
        const line = graph_states.add_line();
        line.scaleFactor = scale_factors[i]/4;  // should be 32 for full range, just for ease of visualisation
        line.offset = 1 - 1/num_lines*i - line.scaleFactor; //0.5;
        line.color = 'hsl(' + (360 / num_lines*i + 0).toString() + ', 40%, 50%)';
        line.linewidth = 2.;
        add_label(label_grid, field_names_states[i], line.color, button=unit_fields_states[i]);
    }

    // //digital graph
    // label_grid = document.querySelector('#digital_grid');
    // num_lines = 24;
    // for (let i = 0; i < num_lines; i++) {
    //     const data_type = i < 16 ? "input" : "output";
    //     const line = graph_digital.add_line();
    //     line.scaleFactor = 0.75/num_lines;
    //     line.offset = 1 - 1/num_lines*i - line.scaleFactor;
    //     line.draw_step = true;
    //     if (i<16) {
    //         line.color = 'hsl(' + (90/num_lines*i).toString() + ', 80%, 60%)';
    //     } else {
    //         line.color = 'hsl(' + (120/num_lines*i+120).toString() + ', 80%, 60%)';
    //     }
    //
    //     var btn = (i < 16) ? null : buttons[i-16];
    //     add_label(label_grid, "digital_" + data_type + "_" + i.toString(), line.color,  box=indicators[i], button=btn);
    // }


    // graphs.addGraph(graph_analog);
    graphs.addGraph(graph_states);
    // graphs.addGraph(graph_digital);
    graphs.start();


    // function btn_clicked(event) {
    //     console.log(event.target.id);
    //     if (ws) {
    //         ws.send(event.target.id);
    //     }
    // }

    ws.onmessage = function(event) {
        if (typeof event.data === 'string') {
            try {
                var packet = JSON.parse(event.data);
            } catch (e) {
                console.log(e);
                console.log(event.data);
                return;
            }

            var gamepads = navigator.getGamepads();
            if (gamepads[0] !== undefined) {
                packet.states[6] = gamepads[0].axes[2] *100 / 2 + 0.5;
                packet.states[7] = gamepads[0].axes[3] *100 / 2 + 0.5;

                ws.send(gamepads[0].axes[2].toFixed(2));
            }


            // console.log(packet);
            // graph_analog.appendPacket(packet.us_start, packet.analog);
            graph_states.appendPacket(packet.us_start, packet.states);
            // graph_digital.appendPacket(packet.us_start, packet.digitalIn.concat(packet.digitalOut));

            // var lbl;
            // // analog charts values
            // for (let i=0; i<8; i++) {
            //     lbl = document.querySelector('.unit_field#analog_' + i.toString());
            //     lbl.textContent = (packet.analog[i] / 2**16 * 3.3).toFixed(2) + ' ' + unit_names_analog[i];
            // }

            // state charts values
            for (let i=0; i<8; i++) {
                lbl = document.querySelector('.unit_field#states_' + i.toString());
                lbl.textContent = (packet.states[i] >= 0 ? '0' : '') + (packet.states[i] / 100).toFixed(2) + ' ' + unit_names_states[i];
            }

            // // digital charts indicators
            // for (let i=0; i<24; i+=1) {
            //     if (i<16) {
            //         if (packet.digitalIn[i]) {
            //                 indicators[i].style = 'background-color: #0F0';
            //         } else {
            //                 indicators[i].style = 'background-color: #000';
            //         }
            //     } else {
            //         if (packet.digitalOut[i-16]) {
            //                 indicators[i].style = 'background-color: #F00';
            //         } else {
            //                 indicators[i].style = 'background-color: #000';
            //         }
            //     }
            // }
        }
    }
});

function start_websocket(ws_port) {
    var ws = new ReconnectingWebSocket("ws://127.0.0.1:" + ws_port.toString());
    ws.reconnectInterval = 500;
    ws.maxReconnectInterval = 1000;
    ws.timeoutInterval = 400;
    return ws;
}
