//////////////////////////////////////////////////////
// UDP - Stuff - (listen for udp packages using dgram)
var arena = new Array();
let arena_rows = 64;
let arena_coloums = 64;
let message_counter = 0;
let rows_in_arena = 0; 

var PORT = 42069;
var HOST = '127.0.0.1';

var dgram = require('dgram');
var udp_recv_server = dgram.createSocket('udp4');

udp_recv_server.on('listening', function() {
  var address = udp_recv_server.address();
  console.log('UDP Server listening on ' + address.address + ':' + address.port);
});

udp_recv_server.bind(PORT, HOST);

//////////////////////////////////////////////////////
// Express - Stuff (expose express server in order to serve index.html)

var express = require('express');
var app = express();
var express_server = app.listen(3000);
console.log('Express is listening on 3000...');
app.use(express.static('public'));

//////////////////////////////////////////////////////
// socket.io - Stuff (build socket.io socket on top of express)

var socket = require('socket.io');
var io = socket(express_server);
io.sockets.on('connection', newConnection);

function newConnection(socket){
    console.log("New client connection: " + socket.id);
    var error = socket.broadcast.emit('arena_update', 4);
    console.log('Error: ' + error);
    //when receiving a new package via udp
    udp_recv_server.on('message', function(message, remote) {
        
        encode_arena(message, remote);
        //broadcast it to every connection
        if(rows_in_arena == arena_rows){ 
            socket.broadcast.emit('arena_update', rows_in_arena);
            rows_in_arena = 0;
            message_counter = 1;
            console.log('Sended an arena to browser!:');
            console.log(arena);
        }
    });
}
//////////////////////////////////////////////////////

//encode the udp back to an two dimensional javascript array
function encode_arena(message, remote){

    console.log('Message counter: ' + message_counter);
    console.log('Rows in Arena: ' + rows_in_arena);
    console.log('Working with bytes: ' + remote.size);
    console.log('Received A' + message.readInt32LE(0) + ' and receives B ' + message.readInt32LE(4));

    if(message_counter == 0){
        arena_rows = message.readInt32LE(4);
        arena_coloums = message.readInt32LE(0);
        console.log('Received rows:' + arena_rows + ' and receives coloumns: ' + arena_coloums);
        message_counter++;
        return;
    }
   
    var arena_row = new Array();

    for(var i = 0; i < arena_coloums; i++){
        try {
            arena_row[i] = message.readInt32LE(i*4);
          } catch (error) {
            console.error(error);
            return;
          }
    }

    arena[(message_counter-1)] = arena_row;
    console.log('Current arena:' + arena);
    message_counter++;
    rows_in_arena++;
     
    return;

}
