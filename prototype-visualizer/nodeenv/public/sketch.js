var socket;

// Appearance settings
const stroke_color = 0;
const stroke_weight = 1;
const scale = 1.5;

function setup() {
  //connect to io socket
  socket = io.connect('http://localhost:3000');

  //prepare canvas
  createCanvas(1000, 1000);
  background(255);
  stroke(stroke_color);
  strokeWeight(stroke_weight);

  //on receiving new event, draw arena
  socket.on('arena_update',
    // When we receive data
    function(data) {
      
      var arena = data;
      var arena_row = new Array();

      console.log('Okay, lets go!');

      for(var i = 0; i < arena.length; i++){
        arena_row = arena[i];
        for(var j = 0; j < arena_row.length; j++){
          
          switch(arena_row[j]) {
            case 0:
              fill(220, 220, 220); //empty-field-white
              break;
            case 1:
              fill(80, 80, 80); //wall-grey
              break;
            case 7:
              fill(1,1,117); //team-blue-player
              break;
            case 8:
              fill(143,3,8); //team-red-player
              break;
            case 2:
              fill(5,250,25); //portal-green
              break;
            case 3:
              fill(2,2,255); //Flag-A-blue
              break;
            case 4:
              fill(255,2,2); //Flag-B-red
              break;
            case 5:
              fill(37,150,90); //Base-A-blue
              break;
            case 6:
              fill(206,14,110); //Base-B-red
              break;
            default:
              fill(245, 66, 230); //Error color
          }
          rect((j*10)*scale, (i*10)*scale, 10*scale, 10*scale);
        } 
      }

    }
  );
}

function draw() {
  // nothing here at the moment
}
