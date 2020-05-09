var ros = new ROSLIB.Ros({ url : 'ws://' + location.hostname + ':9000' });

ros.on('connection', function(){ console.log('websocket: connected'); });
ros.on('error', function(error){ console.log('websocket error: ', error); });
ros.on('close', function(){ console.log('websocket: closed'); });

var ls = new ROSLIB.Topic({
	ros : ros,
	name : '/lightsensors',
	messageType : 'pimouse_ros/LightSensorValues'
});

ls.subscribe(function(message){
	for( e in message ){
		document.getElementById(e).innerHTML = message[e];
	}
});

var cmd = new ROSLIB.Service({
	ros : ros,
	name : '/pimouse_cmd',
	messageType : 'pimouse_control/PiMouseCmd'
});

var cmdOff = new ROSLIB.ServiceRequest({
	on : false,
	run : false,
	face : false,
	forward : 0.0,
	rotation : 0.0
});

var cmdOn = new ROSLIB.ServiceRequest({
	on : true,
	run : false,
	face : false,
	forward : 0.0,
	rotation : 0.0
});

var cmdRun = new ROSLIB.ServiceRequest({
	on : true,
	run : true,
	face : false,
	forward : 0.0,
	rotation : 0.0
});

var cmdFace = new ROSLIB.ServiceRequest({
	on : true,
	run : false,
	face : true,
	forward : 0.0,
	rotation : 0.0
});

var isOn = false;
var isRun = false;
var isFace = false;

function cmdOnWithMotorValues(){
	fw = document.getElementById('vel_fw').innerHTML;
	rot = document.getElementById('vel_rot').innerHTML;
	
	cmdOn.forward = parseInt(fw) * 0.001;
	cmdOn.rotation = 3.141592 * parseInt(rot) / 180;
	cmd.callService(
		cmdOn,
		function(result){
			if(result.isOk){
				document.getElementById('motor_on').className = 'btn btn-danger';
				document.getElementById('motor_run').className = 'btn btn-default';
				document.getElementById('motor_face').className = 'btn btn-default';
				document.getElementById('motor_off').className = 'btn btn-default';
			}
		}
	);
}

document.getElementById('motor_on').addEventListener(
	'click',
	function(e){
		isOn = true;
		isRun = false;
		isFace = false;
		cmdOnWithMotorValues();
	}
);

document.getElementById('motor_run').addEventListener(
	'click',
	function(e){
		if(!isOn){
			return;
		}
		isRun = true;
		isFace = false;
		document.getElementById('vel_fw').innerHTML = 0;
		document.getElementById('vel_rot').innerHTML = 0;
		cmd.callService(
			cmdRun,
			function(result){
				if(result.isOk){
					document.getElementById('motor_on').className = 'btn btn-default';
					document.getElementById('motor_run').className = 'btn btn-danger';
					document.getElementById('motor_face').className = 'btn btn-default';
					document.getElementById('motor_off').className = 'btn btn-default';
				}
			}
		);
	}
);

document.getElementById('motor_face').addEventListener(
	'click',
	function(e){
		if(!isOn){
			return;
		}
		isRun = false;
		isFace = true;
		document.getElementById('vel_fw').innerHTML = 0;
		document.getElementById('vel_rot').innerHTML = 0;
		cmd.callService(
			cmdFace,
			function(result){
				if(result.isOk){
					document.getElementById('motor_on').className = 'btn btn-default';
					document.getElementById('motor_run').className = 'btn btn-default';
					document.getElementById('motor_face').className = 'btn btn-danger';
					document.getElementById('motor_off').className = 'btn btn-default';
				}
			}
		);
	}
);

document.getElementById('motor_off').addEventListener(
	'click',
	function(e){
		if(isOn){
			document.getElementById('vel_fw').innerHTML = 0;
			document.getElementById('vel_rot').innerHTML = 0;
			isOn = false;
			isRun = false;
			isFace = false;
		}
		cmd.callService(
			cmdOff,
			function(result){
				if(result.isOk){
					document.getElementById('motor_on').className = 'btn btn-default';
					document.getElementById('motor_run').className = 'btn btn-default';
					document.getElementById('motor_face').className = 'btn btn-default';
					document.getElementById('motor_off').className = 'btn btn-primary';
				}
			}
		);
	}
);

document.getElementById('touchmotion').addEventListener(
	'click',
	function(e){
		if(!isOn || isRun || isFace){
			return;
		}
		rect = $('#touchmotion')[0].getBoundingClientRect();
		x = e.pageX - rect.left - window.pageXOffset;
		y = e.pageY - rect.top - window.pageYOffset;

		if((x >= (rect.width * 48 / 100)) && (x < (rect.width * 52 / 100))){
			x = rect.width / 2;
		}
		if((y >= (rect.height * 48 / 100)) && (y < (rect.height * 52 / 100))){
			y = rect.height / 2;
		}
		vel_fw = (rect.height / 2 - y) * 3;
		vel_rot = rect.width / 2 - x;
		document.getElementById('vel_fw').innerHTML = parseInt(vel_fw);
		document.getElementById('vel_rot').innerHTML = parseInt(vel_rot);

		cmdOnWithMotorValues();
	}
);

document.getElementById('camstream').data = 'http://'
	+ location.hostname
	+ ':10000/stream?topic=/face';
