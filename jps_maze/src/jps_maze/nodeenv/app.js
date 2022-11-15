//////////////////////////////////////////////////////
// UDP - Stuff - (listen for udp packages using dgram)
var arena = new Array();
let arena_rows = 0;
let arena_coloums = 0;
let message_counter = -1;

var ROS_PORT = process.argv[2];
var BROWSER_PORT = process.argv[3];

var dgram = require('dgram');
var udp_recv_server = dgram.createSocket('udp4');

udp_recv_server.on('listening', function () {
    var address = udp_recv_server.address();
    console.log('UDP Server listening on ' + address.address + ':' + address.port);
});

udp_recv_server.bind(ROS_PORT);

//////////////////////////////////////////////////////
// Express - Stuff (expose express server in order to serve index.html)

var express = require('express');
var app = express();
var express_server = app.listen(BROWSER_PORT);
console.log('Express is listening on '+ BROWSER_PORT + '...');
app.use(express.static('public'));

//////////////////////////////////////////////////////
// socket.io - Stuff (build socket.io socket on top of express)

// BUG Not Working when multiple clients are connected (e.g. Browser refreshed)

var socket = require('socket.io');
var io = socket(express_server);
io.sockets.on('connection', newConnection);

function newConnection(socket) {
    console.log("New client connection: " + socket.id);

    //when receiving a new package via udp
    udp_recv_server.on('message', function (message, remote) {
        //console.log(remote.address + ':' + remote.port +' - ' + remote.size +' - ' + message.readInt32LE(8).toString());
        encode_arena(message, remote);
        //(broadcast) it to every connection
        if (message_counter == arena_rows) {
            socket.broadcast.emit('arena_update', arena);
            console.log('Sended an arena');
            message_counter = -1;
        }
    });
}
//////////////////////////////////////////////////////

//encode the udp back to an two dimensional javascript array
function encode_arena(message, remote) {

    if(message.length == 8 && message_counter == -1) {
        //Begin of arena receive cycle, fetch dimensions
        console.log('Received packages with arena dimensions');
        arena_rows = message.readInt32LE(4);
        arena_coloums = message.readInt32LE(0);
        console.log('Received rows: ' + arena_rows + ' and received coloumns: ' + arena_coloums);
        message_counter++;
        return;
    }else if (message.length == 8 && message_counter != -1){
        //We are already in the arena receive cycle but received a udp package, which doesn't fit our expectations
        console.log("Received unexpected arena dimensions, exiting function, message cnt:", message_counter);
        message_counter = 0;
        return;
    }else{
        //we received an arena row
        var arena_row = new Array();

        for (var i = 0; i < arena_coloums; i++) {
            arena_row[i] = message.readInt32LE(i * 4);
        }

        //console.log('Adding new row at index ' + message_counter);
        arena[message_counter] = arena_row;
        message_counter++;

        return;
    }
}
