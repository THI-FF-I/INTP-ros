//////////////////////////////////////////////////////
// UDP - Stuff - (listen for udp packages using dgram)
var arena = new Array();
let arena_rows = 0;
let arena_coloums = 0;
let message_counter = -1;

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
    
    //when receiving a new package via udp
    udp_recv_server.on('message', function(message, remote) {
        //console.log(remote.address + ':' + remote.port +' - ' + remote.size +' - ' + message.readInt32LE(8).toString());
        encode_arena(message, remote);
        //broadcast it to every connection
        if(message_counter == arena_rows){
            socket.emit('arena_update', arena);
            console.log('Sended an arena');
            message_counter = 0;
        }
    });
}
//////////////////////////////////////////////////////

//encode the udp back to an two dimensional javascript array
function encode_arena(message, remote){

    if(message_counter == -1){
        console.log('Received packages with arena dimensions');
        arena_rows = message.readInt32LE(4);
        arena_coloums = message.readInt32LE(0);
        console.log('Received rows:' + arena_rows + ' and receives coloumns: ' + arena_coloums);
        message_counter++;
        return;
    }

    var arena_row = new Array();

    for(var i = 0; i < arena_coloums; i++){
        arena_row[i] = message.readInt32LE(i*4);
    }

    console.log('Adding new row at index ' + message_counter);
    arena[message_counter] = arena_row;

    message_counter++;

    return;

}
