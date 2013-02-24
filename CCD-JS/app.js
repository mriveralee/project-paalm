
/**
 * Module dependencies.
 */

var express = require('express')
  , http = require('http')
  , path = require('path')
  , CONSTANTS = require('./constants')    //App Constants Vars
  , CONFIG = require('./config');          //App Configuration Vars


//Express App
var app = express();

app.configure(function(){
  app.set('port', process.env.PORT || CONFIG.server_port);
  app.set('views', __dirname + '/views');
  app.set('view engine', CONFIG.template_engine);
  app.use(express.favicon());
  app.use(express.logger('dev'));
  app.use(express.bodyParser());
  app.use(express.methodOverride());
  app.use(express.cookieParser());
  app.use(app.router);
  app.use(express.static(path.join(__dirname, 'public')));
  app.use(app.router);

});


app.configure('development', function(){
  app.use(express.errorHandler());
  app.use(express.bodyParser());
  app.use(express.methodOverride());
  app.use(express.cookieParser());

});


//Run the Server
var server = http.createServer(app);
server.listen(CONFIG.server_port, function() {
  //Log the name of the app from the constants.js file
  console.log(CONSTANTS.app_name);
  
  //Log the port the server is running on - from the config file
  console.log("\n****\n* SERVER RUNNING ON PORT: " + CONFIG.server_port + " *\n****\n");
});


//################ Socket IO ######################//
var io = require('socket.io').listen(server);
//Turn off server-side heartbeat & debug messages from socket.io
// io.set("log level", 0);   

io.sockets.on('connection', function (socket) {
  
  //Sends an event to a connected client with the tag 'my server event'
  socket.emit('my server event', {hello: "I was sent from the server!"});
  
  //Receives an event from the client with the tag 'my client event;'
  socket.on('my client event', function (data) {
       //Log the data sent from the client
       console.log(data);
  });
});


//################ Routes ######################//

//Main route
app.get('/', function(req, res) {
  var templateVars = {
      //Add some template variables
      PAGE_TITLE: "Node (Express 3.0.1) & Socket.io Bootstrap"
  };

  //Render the index.ejs file with any template variables
  res.render('index', templateVars);
});


//Sends hello back when you go to the route
app.get('/coolRoute', function(req, res) {
  res.send("HELLO!");
});

//Leap Sample
app.get('/sample', function(req, res) {
  res.send('Sample.html');
});




