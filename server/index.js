var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var keymap = 000000000;
var drone_ip = 0;
var port = 3000

//var STREAM_SECRET = process.argv[2],
//	STREAM_PORT = process.argv[3] || 3001,
//	WEBSOCKET_PORT = process.argv[4] || 8084,
//	STREAM_MAGIC_BYTES = 'jsmp'; // Must be 4 bytes

//var width = 320,
//	height = 240;
//var socketServer = new (require('ws').Server)({port: WEBSOCKET_PORT});

http.listen(3000, function(){
	console.log('Drone Command Center loading...');
	console.log('listening on *:3000');
	//console.log('Listening for Video Stream on http://avela.ddns.net:'+STREAM_PORT+'/<secret>/<width>/<height>');
	//console.log('Awaiting WebSocket connections on ws://avela.ddns.net:'+WEBSOCKET_PORT+'/');
});

io.on('connection', function(socket) {
	console.log('Client connected');
	io.emit('DroneIP', drone_ip);
	socket.on('DRONEboot', function(ip){
	//	io.emit('IOrequest', 'Drone boot: ' + ip);
		io.emit('DroneIP', ip);
		drone_ip = ip;
	});
	socket.on('KeymapUpdate', function(keys){
		keymap = keys
		io.emit('IOanswer', keys);
		console.log('Server: ' + keys);
	});
	socket.on('DRONErequest', function(msg){
		//io.emit('IOrequest', 'Drone: ' + msg);
		io.emit('IOanswer', keymap);
		//io.emit('IOrequest', 'Keys sent: ' + keymap);
		console.log('Drone: ' + msg);
	});
	socket.on('PIDchange_P', function(pid){
		io.emit('server_pid_p', pid);
		console.log('PID: P  change requested');
	});
	socket.on('PIDchange_I', function(pid){
		io.emit('server_pid_i', pid);
		console.log('PID: I change requested');
	});
	socket.on('PIDchange_D', function(pid){
		io.emit('server_pid_d', pid);
		console.log('PID: D change requested');
	});
	socket.on('calibrate', function(msg){
		io.emit('calibrate', 1);
		console.log('Server: Calibration requested');
	});
	socket.on('PIDmax', function(msg){
		io.emit('server_adjustpidmax', msg);
		console.log('Server: PID max requested: ' + msg);
	});
	socket.on('logger', function(msg){
		console.log('LOG: ' , msg)
	});
	socket.on('disconnect', function(){
		console.log('Client disconnected');
	});
});


//if( process.argv.length < 3 ) {
//	console.log(
//		'Usage: \n' +
//		'node index.js <password> [<stream-port> <websocket-port>]'
//	);
//	process.exit();
//}


// Websocket Server

//socketServer.on('connection', function(socket) {
	// Send magic bytes and video size to the newly connected socket
	// struct { char magic[4]; unsigned short width, height;}
//	var streamHeader = new Buffer(8);
//	streamHeader.write(STREAM_MAGIC_BYTES);
//	streamHeader.writeUInt16BE(width, 4);
//	streamHeader.writeUInt16BE(height, 6);
//	socket.send(streamHeader, {binary:true});

//	console.log( 'New WebSocket Connection ('+socketServer.clients.length+' total)' );
	
//	socket.on('close', function(code, message){
//		console.log( 'Disconnected WebSocket ('+socketServer.clients.length+' total)' );
//	});
//});

//socketServer.broadcast = function(data, opts) {
//	for( var i in this.clients ) {
//		if (this.clients[i].readyState == 1) {
//			this.clients[i].send(data, opts);
//		}
//		else {
//			console.log( 'Error: Client ('+i+') not connected.' );
//		}
//	}
//};


// HTTP Server to accept incomming MPEG Stream
//var streamServer = require('http').createServer( function(request, response) {
//	var params = request.url.substr(1).split('/');

//	if( params[0] == STREAM_SECRET ) {
//		width = (params[1] || 320)|0;
//		height = (params[2] || 240)|0;
//		
//		console.log(
//			'Stream Connected: ' + request.socket.remoteAddress + 
//			':' + request.socket.remotePort + ' size: ' + width + 'x' + height
//		);
//		request.on('data', function(data){
//			socketServer.broadcast(data, {binary:true});
//		});
//	}
//	else {
//		console.log(
//			'Failed Stream Connection: '+ request.socket.remoteAddress + 
//			request.socket.remotePort + ' - wrong secret.'
//		);
//		response.end();
//	}
//}).listen(STREAM_PORT);


