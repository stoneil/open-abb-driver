const net = require('net');


class Robot {

    constructor(ip = '127.0.0.1', port = '5000', wobj = [[0,0,0],[1,0,0,0]], tool = [[0,0,0],[1,0,0,0]], speed = [100,50,50,50], zone = 'z5', toolfile = null, zeroJoints = false, verbose = true) {
        this.ip = ip;
        this.port = port;
        this.v = verbose;

        this.msg = "";
        this.res = "";

        this.wobj = wobj;
        this.tool = tool;
        this.speed = speed;
        this.zone = zone;
        this.tool = undefined;
                
        this.socket = new net.Socket();
        this.socket.on('connect', () => console.log('Connected to server'));
        this.socket.on('close', () => console.log('Disconnected from server'));
        // this.socket.on('data', data => console.log(`Received data: ${data}`));
        this.socket.on('data', this.onDataHandler);

        this.connect();
    }

    // Connections
    connect() {
        this.socket.connect(this.port, this.ip);
    }

    close() {
        this.socket.destroy();
    }

    onDataHandler(data) {
        console.log(`Received data: ${data}`);
        this.res = data;
        // this.waitingForResponse = false;
    }

    // Internal utilities
    checkCoordinates(pos) {
        return pos && pos.length == 2 && pos[0].length == 3 && pos[1].length == 4;
    }

    posToBuffer(pos) {
        return `${pos[0][0]} ${pos[0][1]} ${pos[0][2]} ${pos[1][0]} ${pos[1][1]} ${pos[1][2]} ${pos[1][3]}`;
    }

    sendMessage() {
        if (this.v) console.log("Sending msg: " + this.msg);
        this.socket.write(this.msg);
    }


    // Actions
    setCartesian(pos) {
        if (!pos) return false;
        if (pos.length >= 7) pos = [pos.slice(0, 3), pos.slice(3, 7)];
        if (!this.checkCoordinates(pos)) return false;

        this.msg = `1 ${this.posToBuffer(pos)} #`;
        this.sendMessage();

        return true;
    }

    setJoints(j = [0, 0, 0, 0, 90, 0]) {
        if (j.length != 6) return false;

        this.msg = `2 ${j[0]} ${j[1]} ${j[2]} ${j[3]} ${j[4]} ${j[5]} #`;
        this.sendMessage();

        return true;
    }

    getCartesian() {
        this.msg = '3 #';
        this.sendMessage();

        return true;

        // THIS IS not working
        // this.waitingForResponse = true;
        // while(this.waitingForResponse) { console.log(this.waitingForResponse); }  // artificially block thread until received response

        // let s = this.res.split(' ');
        // if (parseInt(s[0]) != 3 || parseInt(s[1]) != 1) return false;

        // let pos = [
        //     [parsetFloat(s[2]), parsetFloat(s[3]), parsetFloat(s[4])],
        //     [parsetFloat(s[5]), parsetFloat(s[6]), parsetFloat(s[7]), parseFloat(s[8])]
        // ];

        // return pos;
    }

    getJoints() {
        this.msg = '4 #';
        this.sendMessage();

        return true;
    }

    getExternalAxis() {
        this.msg = '5 #';
        this.sendMessage();
        
        return true;
    }

    getRobotInfo() {
        this.msg = '98 #';
        this.sendMessage();

        return true;
    }

    setTool(tool = [[0,0,0],[1,0,0,0]]) {
        if (tool.length >= 7) tool = [tool.slice(0, 3), tool.slice(3, 7)];
        if (!this.checkCoordinates(tool)) return false;

        this.msg = `6 ${this.posToBuffer(tool)} #`;
        this.sendMessage();
        // Should sleep the thrad here... 
        this.tool = tool;

        return true;
    }

    // setToolFile() { }  // TO IMPLEMENT

    getTool() {
        return this.tool;
    }

    setWorkObject(wobj = [[0,0,0],[1,0,0,0]]) {
        if (wobj.length >= 7) wobj = [wobj.slice(0, 3), wobj.slice(3, 7)];
        if (!this.checkCoordinates(wobj)) return false;

        this.msg = `7 ${this.posToBuffer(wobj)} #`;
        this.sendMessage();
        // Should sleep the thrad here... 
        this.wobj = wobj;

        return true;
    }

    setSpeed(speed = [100, 50, 50, 50]) {
        if (speed.length < 4) return false;

        this.msg = `8 ${speed[0]} ${speed[1]} ${speed[2]} ${speed[3]} #`;
        this.sendMessage();

        return true;
    }

    setZone(zoneKey = 'z1', finep = false, manualZone = []) {
        // #zoneKey: uses values from RAPID handbook (stored here in zoneDict), 'z*' 
        // #you should probably use zoneKeys
        // #finep: go to point exactly, and stop briefly before moving on
        // #manualZone = [pzone_tcp, pzone_ori, zone_ori]
        // #pzone_tcp: mm, radius from goal where robot tool center is not rigidly constrained
        // #pzone_ori: mm, radius from goal where robot tool orientation is not rigidly constrained
        // #zone_ori: degrees, zone size for the tool reorientation
        
        let zoneDict = {'z0': [.3,.3,.03], 'z1': [1,1,.1], 'z5': [5,8,.8], 
                        'z10': [10,15,1.5], 'z15': [15,23,2.3], 'z20': [20,30,3], 
                        'z30': [30,45,4.5], 'z50': [50,75,7.5], 'z100': [100,150,15], 
                        'z200': [200,300,30]};

        let zone = [];
        
        if (finep) zone = [0,0,0];
        else if (manualZone.length > 2) zone = manualZone;
        else if (zoneDict[zoneKey]) zone = zoneDict[zoneKey];
        else return false;

        this.msg = `9 ${finep ? 1 : 0} ${zone[0]} ${zone[1]} ${zone[2]} #`;
        this.sendMessage();

        return true;
    }

    addBuffer(pos) {
        // #appends single target to the buffer
        // #move will execute at current speed (which you can change between addBuffer calls)
        if (!pos) return false;
        if (pos.length >= 7) pos = [pos.slice(0, 3), pos.slice(3, 7)];
        if (!this.checkCoordinates(pos)) return false;

        this.msg = `30 ${this.posToBuffer(pos)} #`;
        this.sendMessage();

        return true;
    }

    // setBuffer(posList) { }  // TO IMPLEMENT

    clearBuffer() {
        this.msg = '31 #';
        this.sendMessage();

        return true;
    }

    lenBuffer() {
        this.msg = '32 #';
        this.sendMessage();

        return true;
    }

    executeBuffer() {
        this.msg = '33 #';
        this.sendMessage();

        return true;
    }

    setExternalAxis(axes = [0,0,0,0,0,0]) {
        if (axes.length < 6) return false;

        this.msg = `34 ${axes[0]} ${axes[1]} ${axes[2]} ${axes[3]} ${axes[4]} ${axes[5]} #`;
        this.sendMessage();

        return true;
    }

    // setCircular(circlePoint, endPoint) { }  // TO IMPLEMENT

}


exports.Robot = Robot;
