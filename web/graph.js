/* Inspired by Tso (Peter) Chen's
 * 
 * Photoplethysmograph (Real Time PPG Grapher)
 * 
 * Original license:
 * Absolutely free to use, copy, edit, share, etc.
 * 
 * Written by: Ronny Eichler
 *--------------------------------------------------*/
  
/*
  * Helper function to convert a number to the graph coordinate
  * ----------------------------------------------------------- */
function convertToGraphCoord(l, num){
  return l.graph.height - (num * l.scaleFactor * l.graph.height) - (l.graph.height * l.offset);
}

function random_hsl() {
  return `hsl(${Math.random()*360}, 80%, 75%)`;
}

function GraphCollection() {
  const collection = this;
  collection.graphs = [];
  collection.stop_graphs = false;
  collection.lastDraw = Infinity;

  /*  
    * The call to start the animation
    * ---------------------------------------- */

  // var requestInterval = function (fn, delay) {
  //   var requestAnimFrame = (function () {
  //     return window.requestAnimationFrame || function (callback, element) {
  //       window.setTimeout(callback, 1000 / 60);
  //     };
  //   })(),
  //   start = new Date().getTime(),
  //   handle = {};
  //   function loop() {
  //     handle.value = requestAnimFrame(loop);
  //     var current = new Date().getTime(),
  //     delta = current - start;
  //     if (delta >= delay) {
  //       fn.call();
  //       start = new Date().getTime();
  //     }
  //   }
  //   handle.value = requestAnimFrame(loop);
  //   return handle;
  // };

  collection.start = function() {
    var t = new Date().getTime();
    var delta = (t - collection.lastDraw) ? t > collection.lastDraw : 0;
    var delta_t = t > collection.lastDraw ? t - collection.lastDraw : 0;
    collection.lastDraw = t;

    reqAnimFrame =  window.requestAnimationFrame       ||
                    window.mozRequestAnimationFrame    ||
                    window.webkitRequestAnimationFrame ||
                    window.msRequestAnimationFrame     ||
                    window.oRequestAnimationFrame;
    
    // Recursive call to do animation frames
    if (!collection.stop_graphs) reqAnimFrame(collection.start);
    
    // Draw the frame (with the supplied data buffer)
    if (delta_t < 200) {
      collection.graphs.forEach(graph => {
        graph.draw(delta);
      });
    } else {
      collection.graphs.forEach(graph => {
        graph.flush();
      });
    }

  };

  collection.addGraph = function(graph) {
    collection.graphs.push(graph);
  }
}

function Line(graph, lid) {
  const l = this;
  l.graph           =   graph;
  l.id              =   lid;
  l.linewidth       =   1.;
  l.color           =   `hsl(${Math.random()*360}, ${Math.floor(100 - 3.5 * l.id)}%, ${Math.floor(75 - 10 * (l.id % 4))}%)`;
  l.scaleFactor     =   1;
  l.offset          =   0;
  l.prev_x          =   0;
  l.prev_y          =   0;
  l.dataBuffer      =   [];
  l.dofs            =   0.5;
  l.draw_step       =   false;

  l.draw = function() {
    // If this is first time, draw the first y point depending on the buffer
    if (!l.started) {
      l.prev_x = 0;
      l.prev_y = convertToGraphCoord(l, l.dataBuffer[0]);
      l.started = true;
    }

    const ofs = l.dofs;
    l.graph.context.beginPath();

    // format the line
    l.graph.context.strokeStyle = l.color;
    l.graph.context.lineWidth   = l.linewidth;
    // l.graph.context.lineJoin    = "round";

    // // We first move to the current x and y position (last point)
    // l.graph.context.moveTo(l.graph.current_x, l.graph.current_y);

    for (let i = 0; i < l.dataBuffer.length; i++) {
      // l.graph.context.strokeStyle = random_hsl();
      // Start the drawing
      // Put the new y point in from the buffer
      const x = Math.floor(l.dataBuffer[i][0]) + ofs;
      const y = Math.floor(convertToGraphCoord(l, l.dataBuffer[i][1])) + ofs;

      if (l.prev_x > x) {
        l.prev_x = x;
        l.prev_y = y;
        // Draw the line to the new x and y point
        l.graph.context.moveTo(l.prev_x, l.prev_y);
        continue
      }

      // Draw the line to the new x and y point
      l.graph.context.moveTo(l.prev_x-.5, l.prev_y);

      if (l.draw_step) {
        const dx = x - l.prev_x;
        if (dx >= 2) {
          l.graph.context.lineTo(l.prev_x+dx/2, l.prev_y);
          l.graph.context.lineTo(l.prev_x+dx/2, y);
          l.graph.context.lineTo(l.prev_x+dx, y);
        } else {
          l.graph.context.lineTo(l.prev_x, y);
        }
      } else {
        // l.graph.context.lineTo();
      }
      l.graph.context.lineTo(x, y);
      
      // Create stroke
      l.prev_x = x;
      l.prev_y = y;
      l.graph.context.stroke();
    }
    // Stop the drawing
    l.graph.context.closePath();
    l.dataBuffer.length=0;
  };

}

  /*
   * Constructor for the Graph object
   * ----------------------------------------------------------- */
  function Graph(cid){

    const g = this;
    g.canvas_id       =   cid;
    g.canvas          =   document.getElementById(cid);
    g.canvas.width    =   g.canvas.offsetWidth;
    g.canvas.height   =   g.canvas.offsetHeight;
    g.context         =   g.canvas.getContext("2d");
    g.width           =   g.canvas.width;
    g.height          =   g.canvas.height;
    g.bg_color        =   "rgba(0, 0, 0, ";
    g.white_out       =   g.width * 0.02;
    g.fade_out        =   g.width * 0.05;
    g.fade_opacity    =   0.15;
    g.current_x       =   Infinity;
    g.current_y       =   0;
    g.erase_x         =   null;
    g.x_range         =   5000;  // ms

    g.stop_graph      =   false;
    
    g.graphStarted    =   false;
    g.t0 = 0;

    g.lines           =   [];
   
    g.add_line = function() {
      let line = new Line(g, g.lines.length);
      g.lines.push(line);
      return line;
    };

    g.appendPacket = function(us, data) {
      const x = g.us_to_x(us);
      if (data.length !== g.lines.length) {
        console.log('Wrong data length.', data.length, g.lines.length);
        return;
      }

      for (let i = 0; i < g.lines.length; i++) {
          g.lines[i].dataBuffer.push([x, data[i]]);
      }

      g.current_x = x;
    };

    g.flush = function() {
      g.context.clearRect(0, 0, g.width, g.height);
      for (let i = 0; i < g.lines.length; i++) {
          g.lines[i].dataBuffer.length = 0;
          g.lines[i].started = false;
      }
    };

    g.t_to_x = function(t) {
      return Math.floor((t - g.t0) / g.msPerPixel);
    };

    g.us_to_x = function(us) {
      return (us/1000 % g.x_range) * (g.width / g.x_range);
    };

    g.draw = function(delta) {
      // Circle back the draw point back to zero when needed (ring drawing)
      // this resets the start time, too.
      g.current_x += (delta / g.width * g.x_range);
      if (g.current_x > g.width) {
        g.current_x = 0;
      }

      // "White out" a region before the draw point
      for(let i = 0; i < g.white_out ; i++){
        g.erase_x = (g.current_x + i) % g.width;
        g.context.clearRect(g.erase_x, 0, 1, g.height);
        // g.context.fillStyle=g.bg_color + "1)";
        // g.context.fillRect(g.erase_x, 0, 1, g.height);
      }
      
      // "Fade out" a region before the white out region
      for(let i = g.white_out ; i < g.fade_out ; i++ ){
        g.erase_x = (g.current_x + i) % g.width;
        g.context.fillStyle=g.bg_color + g.fade_opacity.toString() + ")";
        g.context.fillRect(g.erase_x, 0, 1, g.height);
      }

      // Draw the individual lines
      const cx = g.current_x;
      g.lines.forEach(line => {
        g.current_x = cx;
        line.draw();
      });
  }
}