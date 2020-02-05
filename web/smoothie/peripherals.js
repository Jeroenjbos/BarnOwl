const fpsLimit = 60;
const millisPerPixel = 15;
const plotDelay = 150;

let startTime = new Date().getTime();
var last_timestamp = Infinity;

var indicators = [];
for (i=0; i<24; i++) {
    ind = document.createElement('div');
    ind.className = "indicator";
    ind.id = "digital"+i.toString();
    indicators.push(ind);
}

var buttons = [];
for (i=0; i<8; i++) {
      btn = document.createElement('button');
      btn.className = "toggle_button";
      btn.id = "digital"+(16+i).toString();
      btn.onclick = btn_clicked;
      buttons.push(btn);
}

function add_label(grid, channel_name, color, box=null, button=null) {
      div = document.createElement("div");
      div.className = "label";
      div.id = channel_name;
      div.style = "background-color: " + color;
      tn = document.createTextNode(channel_name);

      if (box != null) {
          div.appendChild(box);
      }
      div.appendChild(tn);
      if (button != null) {
            div.appendChild(button);
      }
      grid.appendChild(div);
      return div;
}

function randomColor(brightness){
      function randomChannel(brightness){
            var r = 255-brightness;
            var n = 0|((Math.random() * r) + brightness);
            var s = n.toString(16);
            return (s.length==1) ? '0'+s : s;
      }
      return '#' + randomChannel(brightness) + randomChannel(brightness) + randomChannel(brightness);
}

var analog_canvas = document.getElementById('analog-chart');
var analogChart = new SmoothieChart({millisPerPixel:millisPerPixel,
      interpolation:'step',
      grid:{verticalSections:5,
            borderVisible:false},
      tooltip:true,
      maxValue:65536,
      minValue:0,
      labels:{disabled:true},
      limitFPS:fpsLimit,
      });
analogChart.streamTo(analog_canvas, plotDelay);

// Populate analog channels and labels
let analogLines = []
label_grid = document.querySelector('.label-grid#analog');
for (let i=0; i<8; i+=1) {
      var ts = new TimeSeries;
      let color = randomColor(50);

      analogChart.addTimeSeries(ts, {lineWidth:1.5, strokeStyle: color});
      analogLines.push(ts);

      add_label(label_grid, 'analog_input_'+i.toString(), color);
}

// add line for microseconds
var ts = new TimeSeries;
analogChart.addTimeSeries(ts, {lineWidth:1, strokeStyle: '#FFFFFF'});
analogLines.push(ts);
add_label(label_grid, 'timestamp_us', '#FFFFFF');

// add line for microseconds
var ts = new TimeSeries;
analogChart.addTimeSeries(ts, {lineWidth:1, strokeStyle: '#FF00FF'});
analogLines.push(ts);
add_label(label_grid, 'gathertime_us', '#FF00FF');


var digital_canvas = document.getElementById('digital-chart');
var digitalChart = new SmoothieChart({millisPerPixel:millisPerPixel,
      interpolation:'step',
      grid:{verticalSections:16,
            borderVisible:false},
      tooltip:false,
      maxValue:24*1.5,
      minValue:0,
      labels:{disabled:true},
      limitFPS:fpsLimit,
      });
digitalChart.streamTo(digital_canvas, plotDelay);

let digitalLines = []
label_grid = document.querySelector('.label-grid#digital');
for (let i=0; i<16; i+=1) {
      var ts = new TimeSeries;
      let color = randomColor(80);

      digitalChart.addTimeSeries(ts,
            {lineWidth:1.5, 
            strokeStyle: color});
      digitalLines.push(ts);

      add_label(label_grid, "digital_input_"+i.toString(), color, box=indicators[i]); // 
}

for (let i=0; i<8; i+=1) {
      var ts = new TimeSeries;
      let color = randomColor(50);

      digitalChart.addTimeSeries(ts, {lineWidth:1.5, strokeStyle: color});
      digitalLines.push(ts);

      add_label(label_grid, "digital_output_"+i.toString(), color, box=indicators[16+i], button=buttons[i]); // 
}

function btn_clicked(event) {
      console.log(event.target.id);
      if (ws) {
            ws.send(event.target.id);
      }
}

var ws = new ReconnectingWebSocket("ws://127.0.0.1:5678/");
ws.reconnectInterval = 500;
ws.maxReconnectInterval = 1000;
ws.timeoutInterval = 400;
var disp = false;

ws.onmessage = function (event) {
    if (typeof event.data === 'string') {
        try {
            var packet = JSON.parse(event.data);
        } catch (e) {
            console.log(e);
            console.log(event.data);
            return;
        }

        if (packet.us_start < last_timestamp) {
            console.log('timestamp ran over, resetting start time.')
            last_timestamp = packet.us_start;
            startTime = new Date().getTime();
            console.log('last_timestamp: ' + last_timestamp.toString());
            console.log('start_timestamp: ' + (startTime + Math.floor((packet.us_start-last_timestamp) / 1000)).toString());
            console.log('dt: ' + startTime.toString());
        }
        ts = startTime + Math.floor((packet.us_start-last_timestamp) / 1000)-200;
        ts_now = new Date().getTime();
        console.log(ts, ts_now, ts-ts_now);

        // add data to the charts

        // analog data
        analogLines[8].append(ts, packet.us_start / (2**16));
        analogLines[9].append(ts, (packet.us_end-packet.us_start)*100);
        for (let i=0; i<8; i+=1) {
                analogLines[i].append(ts, packet.analog[i]);
        }

        // digital charts
        for (let i=0; i<24; i+=1) {
            if (i<16) {
                  digitalLines[i].append(ts, packet.digitalIn[i] + (35 - 1.5*i));
                  if (packet.digitalIn[i]) {
                        indicators[i].style = 'background-color: #0F0';
                  } else {
                        indicators[i].style = 'background-color: #000';
                  }
            } else {
                  digitalLines[i].append(ts, packet.digitalOut[i-16] + (35 - 1.5*i));
                  if (packet.digitalOut[i-16]) {
                        indicators[i].style = 'background-color: #F00';
                  } else {
                        indicators[i].style = 'background-color: #000';
                  }
            }
        }

    }
};
