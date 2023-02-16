
var connectTo       = "F5:CB:47:42:E3:BB public";
var primServiceUUID = "19b10000-0000-537e-4f6c-d104768a1214";
var tempCharactUUID = "19b10000-2001-537e-4f6c-d104768a1214";
var co2CharactUUID  = "19b10000-9002-537e-4f6c-d104768a1214";
var iaqCharactUUID  = "19b10000-9001-537e-4f6c-d104768a1214";
var humCharactUUID  = "19b10000-3001-537e-4f6c-d104768a1214";
var preCharactUUID  = "19b10000-4001-537e-4f6c-d104768a1214";
var gasCharactUUID  = "19b10000-9003-537e-4f6c-d104768a1214";
var accCharactUUID  = "19b10000-9004-537e-4f6c-d104768a1214";


var tmpAvg = 0;
var co2Avg = 0;
var iaqAvg = 0;
var preAvg = 0;
var gasAvg = 0;

var tmpBuf = 0;
var co2Buf = 0;
var iaqBuf = 0;
var preBuf = 0;
var gasBuf = 0;

var tmpComp = -5;  // temperature calibration

var gatt;
var history = new Float32Array(64);
var choice = 0;
var True = 1;
var False = 0;
var altCalib = True;
var firstTime = True;

require("Font7x11Numeric7Seg").add(Graphics);  // Load fonts


// Main menu
var mainmenu = {
  "" : {
    "title" : "Button=Exit"  },
  "Level"  : function() { choice = 1;  bt_connect(); },
  "Exit "  : function() {load();},  };

g.clear();
E.showMenu(mainmenu);


// *************************
// ****** FUNCTIONS ********
// *************************

function graph(data) {

  // quickly move all elements of history back one
  history.set(new Float32Array(history.buffer,4));
  history[history.length-1] = data;

  g.clear();
  // Draw Graph
  var r = require("graph").drawLine(g, history, {
    axes : true,
    // miny : 0,
    gridy : 5,
    padx  : 2,
    pady  : 2,
    title: ""
  });

  g.setFontAlign(1,-1);
  g.setFont("Vector",14);

  g.drawString(data, r.x+r.w, r.gety(data));
  g.flip();  // Update the screen
}


// Displaying data
function show_acc(acc) {

  g.setColor(0,0,0);
  g.clear();
  
  showMsg(acc.toFixed(2) + "°", 30,  15, g.getHeight()/2 - 30);
}

function showMsg(text, fntSize, x, y) {
  g.setFont("Vector", fntSize);
  g.drawString(text, x, y);
}

function message(text1, text2, text3, fntSize, x, y) {
  g.clear();
  showMsg(text1, fntSize, x, y-(fntSize * 1.1));
  showMsg(text2, fntSize, x, y);
  showMsg(text3, fntSize, x, y+(fntSize * 1.1));
}



function alarm2() {
  Bangle.setLCDPower(1);
  g.clear();
  var t1 = getTime() + 5;   // showing message for how many seconds
  while(getTime() < t1) {
    message("    FALL", " DETECTED !", "", 25, 2, g.getHeight()/2 - 10);
    g.flip();
  }
  
  var t2 = getTime() + 10;  // showing message for how many seconds
  var counter = 0;
  while(getTime() < t2) {
    g.clear();
    counter = t2 - getTime();
    showMsg("Calling help in" , 25,                   2, g.getHeight()/2 - 50);
    showMsg(counter.toFixed(0), 40, g.getWidth()/2 - 20, g.getHeight()/2 -  0);
    showMsg(" seconds"        , 35,                   2, g.getHeight()/2 + 50);
    g.flip();
  }
  
  var t3 = getTime() + 5;   // showing message for how many seconds
  while(getTime() < t3) {
    message("CALLING", "EMERGENCY", "NUMBER", 25, 15, g.getHeight()/2 - 10);
    g.flip();
  }
  g.clear();
}

function alarm() {  
  for (i = 0; i<10; i++) {
    Bangle.setLCDPower(1);
    for (j = 0; j<20; j++) {
      message("    FALL", " DETECTED !", "", 25, 2, g.getHeight()/2 - 10);
      Bangle.setLCDPower(0);
    }
    g.flip();
    Bangle.setLCDPower(1);
  }
  g.setColor(0,0,0);
}

function bt_connect() {
  message(" Connecting", " the dots ...", "", 25, 2, g.getHeight()/2 - 10);

  setWatch(function() {
    E.showMenu(mainmenu);
    choice = 0;
  }, BTN, {edge:"rising", debounce:200, repeat:false});

  NRF.connect(connectTo).then(function(g) {
    gatt = g;
    return gatt.getPrimaryService(primServiceUUID);
  }).then(function()        {    return gatt.getPrimaryService(primServiceUUID);
  }).then(function(service) {    return service.getCharacteristic(accCharactUUID);
  }).then(function(ch)      {
      ch.on('characteristicvaluechanged', function(acc) {

      acc = acc.target.value.buffer;
      dshAcc = (acc[1] * 256 + acc[0]);    
      if (dshAcc > 50) {
        console.log("FALL DETECTED!");
        alarm2();
      }

      var date = new Date();
      var timeStr = require("locale").time(date,1);
      message(timeStr, "", "", 50, g.getWidth()/2-70, g.getHeight() / 2 + 30);   //fake watch

      //if (choice == 1) show_acc(dshAcc);

      logTxt = dshAcc.toFixed(4);
      console.log(logTxt);

      });
    return ch.startNotifications();

  }).then(function() {
    console.log("Subscribed to notifications...");
  }).catch(function(e) {
    E.showMessage(e.toString(), "ERROR");
    console.log(e);
  });
}