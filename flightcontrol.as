// -----------------------------------------------------------------------------
// flightcontrol.as
// nandOS
// Created by Stjepan Stamenkovic.
// -----------------------------------------------------------------------------

// Basic attitude controls, reads actual navigation data but not used yet

// ---------------------------------------------------------

//   Guidance: Generate desired process variables
// Navigation: Prepare sensor data as measured process variables
//    Control: Calculate control values, impose restrictions, smoothing

// Input: Process variables (measured, desired) -> error, command variables -> control variables 

// PID control - Proportional-Integral-Derivative control
// previous_error = 0
// integral = 0 
// start:
//	error = setpoint - measured_value
//	integral = integral + error*dt
//	derivative = (error - previous_error)/dt
//	output = Kp*error + Ki*integral + Kd*derivative
//	previous_error = error
//	wait(dt)
//	goto start

// ---------------------------------------------------------

class NavigationData
{
	vec3 position;
	vec3 orientationAngles;
	vec3 linearVelocity;
	vec3 angularVelocity;	

	quat toGlobalAngles;
	quat toLocalAngles;
	vec3 localAngularVelocity;

	// -----------------------------------------------------
	
	vec3 ToGlobalAngles(vec3 localAngles)
	{
		return toGlobalAngles * quat(localAngles);
	}
	
	vec3 ToLocalAngles(vec3 globalAngles)
	{
		return toLocalAngles * quat(globalAngles);
	}
	
	// -----------------------------------------------------
		
	string opImplConv()
	{
		string s = "NavigationData:\n";
		s += "    Position: " + position + "\n";
		s += " Orientation: " + degrees(orientationAngles) + "\n";
		s += " LinVelocity: " + linearVelocity + "\n";
		s += " AngVelocity: " + degrees(angularVelocity) + "\n";
		s += "LAngVelocity: " + degrees(localAngularVelocity) + "\n";
		
		return s;
	}
}

NavigationData navigationData;

// ---------------------------------------------------------

namespace ControlData
{
	enum Axis
	{
		Axis_X = 0,
		Axis_Y,
		Axis_Z
	}
	
	Axis GetDominantAxis(vec3 v)
	{
		vec3 vAbs = abs(v);
		
		if  (vAbs.x > vAbs.y)
		{
			return vAbs.x > vAbs.z ? Axis_X : Axis_Z;
		}
		else
		{
			return vAbs.y > vAbs.z ? Axis_Y : Axis_Z;
		}
	}
}

class ControlData
{
	// Guidance values
	vec3 desiredDestination;
	vec3 desiredOrientationAngles;

	// Derived values, take NavigationData into account
	vec3 desiredLocalAngularVelocity;
	float desiredMainThrust;
	
	// Final deltas
	vec3 angularVelocityDelta;
	float mainThrustDelta;
	
	// -----------------------------------------------------
		
	string opImplConv()
	{
		string s = "ControlData:\n";
		s += "Desired Destination: " + desiredDestination + "\n";
		s += "Desired Orientation: " + degrees(desiredOrientationAngles) + "\n";
		s += "Desired AngVelocity: " + degrees(desiredLocalAngularVelocity) + "\n";
		s += " Desired MainThrust: " + desiredMainThrust + "\n";
		s += "  AngVelocity Delta: " + degrees(angularVelocityDelta) + "\n";
		s += "   MainThrust Delta: " + mainThrustDelta + "\n";
		
		return s;
	}
}

ControlData controlData;

// ---------------------------------------------------------

// Sidestick file descriptor
int fdSidestick;

// Thruster device file descriptors
vector<int> fdAttitudeThruster;

// Thruster groups
vector<int> thrustersPitchPos;
vector<int> thrustersPitchNeg;
vector<int> thrustersYawPos;
vector<int> thrustersYawNeg;
vector<int> thrustersRollPos;
vector<int> thrustersRollNeg;

// Main engine device
int fdMainEngine;

// Navigation device
int fdNavigation;

// ---------------------------------------------------------

void controlGroup(const vector<int> &in fds, float power)
{
	for (uint i = 0; i < fds.size(); i++)
	{
		vector<var> controlCode = {Control_Thruster_Power, power};
		write(fds[i], controlCode);
	}
}

void controlEngine(float power)
{
	vector<var> controlCode = {Control_Thruster_Power, power};
	write(fdMainEngine, controlCode);
}

// ---------------------------------------------------------

// How a thruster affects an axis
enum ThrusterEffect
{
	Thrust_None = 0,	// No effect
	Thrust_Positive,	// Rotate around axis
	Thrust_Negative		// Rotate against axis
}

// Open the thruster device and associate it with 
// contributing to positive/negative pitch/yaw/roll
bool initialiseThruster(string devicePath,
	ThrusterEffect pitch, ThrusterEffect yaw, ThrusterEffect roll)
{
	int fd = open(devicePath, O_WRONLY);
	
	if (fd == -1)
	{
		return false;
	}
	
	// Register thruster with appropriate group
	
	if (pitch == Thrust_Positive)
	{
		thrustersPitchPos.push_back(fd);
	}
	else if (pitch == Thrust_Negative)
	{
		thrustersPitchNeg.push_back(fd);
	}
	
	if (yaw == Thrust_Positive)
	{
		thrustersYawPos.push_back(fd);
	}
	else if (yaw == Thrust_Negative)
	{
		thrustersYawNeg.push_back(fd);
	}
	
	if (roll == Thrust_Positive)
	{
		thrustersRollPos.push_back(fd);
	}
	else if (roll == Thrust_Negative)
	{
		thrustersRollNeg.push_back(fd);
	}
	
	// Register thruster with general group that's closed at shutdown
	
	fdAttitudeThruster.push_back(fd);

	return true;
}

// ---------------------------------------------------------

// TODO add to standard headers
void perror(string str)
{
	log(str + " : errno (TODO)");
}

bool initialiseDevices()
{
	int fdMode;
	
	// --------------------------------------------------------
	// IN
	// --------------------------------------------------------

	fdSidestick = open("/dev/sidestick", O_RDONLY);

	if (fdSidestick == -1)
	{
		perror("Sidestick");
		return false;
	}

	fdMode = fcntl(fdSidestick, F_GETFL);
	fcntl(fdSidestick, F_SETFL, fdMode | O_NONBLOCK);

	// ---
	
	fdNavigation = open("/dev/nav", O_RDONLY);
	
	if (fdNavigation == -1)
	{
		perror("Navigation");
		return false;
	}
	
	fdMode = fcntl(fdNavigation, F_GETFL);
	fcntl(fdNavigation, F_SETFL, fdMode | O_NONBLOCK);
	
	// --------------------------------------------------------
	// OUT
	// --------------------------------------------------------

	// Thruster devices specify contribution to: Pitch, Yaw, Roll
	
	// Front Thrusters
	
	if (!initialiseThruster("/dev/thruster0",
		Thrust_Negative, Thrust_None, Thrust_Positive))
	{
		perror("Left Top Front : Up");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster1",
		Thrust_None, Thrust_Negative, Thrust_Negative))
	{
		perror("Left Top Front : Left");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster2",
		Thrust_None, Thrust_Negative, Thrust_Positive))
	{
		perror("Left Bottom Front : Left");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster3",
		Thrust_Positive, Thrust_None, Thrust_Negative))
	{
		perror("Left Bottom Front : Down");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster4",
		Thrust_Positive, Thrust_None, Thrust_Positive))
	{
		perror("Right Bottom Front : Down");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster5",
		Thrust_None, Thrust_Positive, Thrust_Negative))
	{
		perror("Right Bottom Front : Right");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster6",
		Thrust_None, Thrust_Positive, Thrust_Positive))
	{
		perror("Right Top Front : Right");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster7",
		Thrust_Negative, Thrust_None, Thrust_Negative))
	{
		perror("Right Top Front : Up");
		return false;
	}
	
	// NOTE /dev/thruster8 is the central engine responsible for forward propulsion
	
	// Back Thrusters
	
	if (!initialiseThruster("/dev/thruster9",
		Thrust_Positive, Thrust_None, Thrust_Positive))
	{
		perror("Left Top Back : Up");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster10",
		Thrust_None, Thrust_Positive, Thrust_Negative))
	{
		perror("Left Top Back : Left");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster11",
		Thrust_None, Thrust_Positive, Thrust_Positive))
	{
		perror("Left Bottom Back : Left");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster12",
		Thrust_Negative, Thrust_None, Thrust_Negative))
	{
		perror("Left Bottom Back : Down");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster13",
		Thrust_Negative, Thrust_None, Thrust_Positive))
	{
		perror("Right Bottom Back : Down");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster14",
		Thrust_None, Thrust_Negative, Thrust_Negative))
	{
		perror("Right Bottom Back : Right");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster15",
		Thrust_None, Thrust_Negative, Thrust_Positive))
	{
		perror("Right Top Back : Right");
		return false;
	}
	
	if (!initialiseThruster("/dev/thruster16",
		Thrust_Positive, Thrust_None, Thrust_Negative))
	{
		perror("Right Top Back : Up");
		return false;
	}

	// --------------------------------------------------------
	
	fdMainEngine = open("/dev/thruster8", O_WRONLY);

	if (fdMainEngine == -1)
	{
		log("Failed opening the main engine device!");
		return false;
	}
	
	// --------------------------------------------------------
	
	// log("### Sidestick : " + fdSidestick);
	// log("### Engine : " + fdMainEngine);
	// for (uint i = 0; i < fdAttitudeThruster.size(); i++)
	// {
		// log("### Thruster" + i + " : " + fdAttitudeThruster[i]);
	// }
	
	return true;
}

bool shutdownDevices()
{
	if (close(fdSidestick) == -1)
	{
		perror("Failed closing sidestick");
		return false;
	}
	
	for (uint i = 0; i < fdAttitudeThruster.size(); i++)
	{
		if (close(fdAttitudeThruster[i]) == -1)
		{
			perror("Failed closing attitude thruster" + i);
			return false;
		}
	}
	
	if (close(fdMainEngine) == -1)
	{
		perror("Failed closing main engine");
		return false;
	}
	
	return true;
}

// ---------------------------------------------------------

void updateNavigation()
{
	// Incoming navigation data:
	// [0] vec3 : position
	// [1] vec3 : orientationAnglesRadian
	// [2] vec3 : linearVelocity
	// [3] vec3 : angularVelocity
	vector<var> navigationIn;
	
	ssize_t r = read(fdNavigation, navigationIn, 32);
	
	if (r == -1)
	{
		// TODO Check error code
		return;
	}
	
	// -----------------------------------------------------
	
	navigationData.position = navigationIn[0];
	navigationData.orientationAngles = navigationIn[1];
	navigationData.linearVelocity = navigationIn[2];
	navigationData.angularVelocity = navigationIn[3];
	
	quat qOrientation = quat(navigationData.orientationAngles);
	navigationData.toGlobalAngles = qOrientation;
	navigationData.toLocalAngles = inverse(qOrientation);
		
	navigationData.localAngularVelocity =
		navigationData.ToLocalAngles(navigationData.angularVelocity);
	
	// -----------------------------------------------------
	
	// Log navigation data for debugging
	// log(navigationData);
}

// ---------------------------------------------------------
	
void updateControls()
{
	// Incoming sidestick data: 
	// [0] vec4 : normalized axes
	vector<var> sidestickIn;
	
	ssize_t r = read(fdSidestick, sidestickIn, 32);
	
	if (r == -1)
	{
		// No new data on sidestick, continue with last control value
		return;
	}
	else
	{
		vec4 axes = sidestickIn[0];
		
		// The sidestick position changes the desired orientation and destination
			
		// NOTE For now just map control directly onto thruster groups, manual control
		controlData.angularVelocityDelta = vec3(axes.x, axes.y, axes.z);
		// controlData.mainThrustDelta = axes.w;
	}
	
	// -----------------------------------------------------

	// TODO Calculate actually desired velocities from destination/orientation
}

// ---------------------------------------------------------

void controlThrusters()
{
	// Final deltas
	vec3 angularVelocityDeltaAbs = abs(controlData.angularVelocityDelta);

	// Map X sidestick to X-thruster (pitch)
	if (angularVelocityDeltaAbs.x > 0)
	{
		// log("Thrusters x: " + controlData.angularVelocityDelta.x);
	
		if (controlData.angularVelocityDelta.x > 0)
		{
			// log("positive pitch");
			controlGroup(thrustersPitchPos, angularVelocityDeltaAbs.x);
			controlGroup(thrustersPitchNeg, 0);
		}
		else
		{
			// log("negative pitch");
			controlGroup(thrustersPitchPos, 0);
			controlGroup(thrustersPitchNeg, angularVelocityDeltaAbs.x);
		}
	}
	
	// Map Y sidestick to Y-thruster (yaw)
	if (angularVelocityDeltaAbs.y > 0)
	{
		// log("Thrusters y: " + controlData.angularVelocityDelta.y);
	
		if (controlData.angularVelocityDelta.y > 0)
		{
			// log("positive yaw");
			controlGroup(thrustersYawPos, angularVelocityDeltaAbs.y);
			controlGroup(thrustersYawNeg, 0);
		}
		else
		{	
			// log("negative yaw");
			controlGroup(thrustersYawPos, 0);
			controlGroup(thrustersYawNeg, angularVelocityDeltaAbs.y);
		}
	}
	
	// Map Z sidestick to Z-thruster (roll)
	if (angularVelocityDeltaAbs.z > 0)
	{
		// log("Thrusters z: " + controlData.angularVelocityDelta.z);
	
		if (controlData.angularVelocityDelta.z > 0)
		{	
			// log("positive roll");
			controlGroup(thrustersRollPos, angularVelocityDeltaAbs.z);
			controlGroup(thrustersRollNeg, 0);
		}
		else
		{
			// log("negative roll");
			controlGroup(thrustersRollPos, 0);
			controlGroup(thrustersRollNeg, angularVelocityDeltaAbs.z);
		}
	}
	
	// TODO Value 0? Controls oscillate around it probably anyway...
	
	if (length(angularVelocityDeltaAbs) < 0.001)
	{
		// Stop everything
		controlGroup(fdAttitudeThruster, 0);
	}
	
	// -----------------------------------------------------
	
	float mainThrustDeltaAbs = abs(controlData.mainThrustDelta);
	
	// Main engine
	if (mainThrustDeltaAbs > 0)
	{
		controlEngine(controlData.mainThrustDelta);
	}
}

// ---------------------------------------------------------

int main(uint argc, vector<var> &in argv)
{
	log("Starting Flight Controller");
	
	log("PID=" + getpid() + " PPID=" + getppid());

	// --------------------------------------------------------

	if (!initialiseDevices())
	{
		log("Failed to initialise device nodes!");
		return 1;
	}
	
	// --------------------------------------------------------
	
	// Input loop
	while(true)
	{
		// TODO select() input
		
		// Fetch latest navigation data for course corrections
		updateNavigation();
		
		// Determine desired attitude and other values, calculate thruster values
		updateControls();
		
		// Apply thruster controls
		controlThrusters();
	}

	// --------------------------------------------------------
	
	// Release drivce nodes

	if (!shutdownDevices())
	{
		log("Failed to release device nodes!");
		return 3;
	}
	
	// --------------------------------------------------------
	
	return 0;
}