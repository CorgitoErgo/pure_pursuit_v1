#include "main.h"

void disabled() {}
void competition_initialize() {}

void serialRead(void* params){
    int sussy;
    double killed;
    double why;
    sussy = 1;
    int getfucked;
    //vexGenericSerialEnable(SERIALPORT - 1, 0);
    //vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    pros::delay(10);
    pros::screen::set_pen(COLOR_BLUE);
    double distX, distY = 0;
    while(true){
        uint8_t buffer[256];
        int bufLength = 256;
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
        if(nRead >= 0){
            std::stringstream dataStream("");
            bool recordOpticalX, recordOpticalY = false;
            for(int i=0;i<nRead;i++){
                char thisDigit = (char)buffer[i];
                if(thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X'||  thisDigit == 'C'||  thisDigit == 'Y'){
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                if(thisDigit == 'C'){
                    recordOpticalX = false;
                    dataStream >> distX;
                    pros::lcd::print(1, "Optical Flow:");
                    pros::lcd::print(2, "distX: %.2lf", distX/100);
                    dataStream.str(std::string());
                }
                if(thisDigit == 'D'){
                    recordOpticalY = false;
                    dataStream >> distY;
                    global_distY = distY/100;
                    pros::lcd::print(3, "distY: %.2lf", distY/100);
                    dataStream.str(std::string());
                }
                if (recordOpticalX) dataStream << (char)buffer[i];
                if (recordOpticalY) dataStream << (char)buffer[i];
                if (thisDigit == 'X') recordOpticalX = true;
                if (thisDigit == 'Y') recordOpticalY = true;
            }
        }
        pros::Task::delay(25);
    }
}

void brake(){
    lf.brake();
    lm.brake();
    lb.brake();

    rf.brake();
    rm.brake();
    rb.brake();
    pros::delay(2);
}

double bound_value(double value){
    if (value > MAX_RPM) return MAX_RPM;
    if (value < -MAX_RPM) return -MAX_RPM;
    return value;
}

class PurePursuit {
public:
    PurePursuit(std::vector<std::pair<double, double>> waypoints, double lookahead_distance)
        : waypoints(waypoints), lookahead_distance(lookahead_distance) {}

    int find_closest_waypoint(double x, double y) {
        double min_distance = std::numeric_limits<double>::max();
        int closest_index = -1;

        for (int i = 0; i < waypoints.size(); i++) {
            double distance = std::hypot(waypoints[i].first - x, waypoints[i].second - y);
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }

        return closest_index;
    }

    std::pair<double, double> get_target_point(double x, double y) {
        int closest_index = find_closest_waypoint(x, y);
        int target_index = closest_index;

        while (target_index < waypoints.size() &&
               std::hypot(waypoints[target_index].first - x, waypoints[target_index].second - y) < lookahead_distance) {
            target_index++;
        }

        if (target_index > closest_index) {
            return waypoints[target_index - 1];
        }
        return waypoints[closest_index];
    }

    double calculate_steering_angle(double x, double y, double heading) {
        auto target_point = get_target_point(x, y);
        double angle_to_target = std::atan2(target_point.second - y, target_point.first - x);
        double steering_angle = angle_to_target - heading;

        return steering_angle;
    }

private:
    std::vector<std::pair<double, double>> waypoints;
    double lookahead_distance;
};

class tankBase {
public:
    // Constructor to initialize motors and IMU
    tankBase()
        : lf(LEFT_FRONT_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES),
          lm(LEFT_MIDDLE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES),
          lb(LEFT_BACK_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES),
          rf(RIGHT_FRONT_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES),
          rm(RIGHT_MIDDLE_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES),
          rb(RIGHT_BACK_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES),
          imu_sensor(IMU_SENSOR_PORT) // Initialize the IMU sensor
    {
        reset_odometry(); // Initialize position
    }

    void update_odometry() {
        double left_distance = (lf.get_position() + lm.get_position() + lb.get_position()) / 3.0 * PosConvert; // Average left side distance
        double right_distance = (rf.get_position() + rm.get_position() + rb.get_position()) / 3.0 * PosConvert; // Average right side distance

        // Average distance
        double distance_traveled = (left_distance + right_distance) / 2.0;

        // Get the heading from the IMU
        theta = imu_sensor.get_rotation() * M_PI / 180.0; // Convert degrees to radians

        // Update the position based on the distance traveled and heading
        x += distance_traveled * cos(theta);
        y += distance_traveled * sin(theta);
    }

    double get_x() const { return x; }
    double get_y() const { return y; }
    double get_heading() const { return theta; }

    void reset_odometry() {
        // Reset motor positions to zero
        lf.tare_position();
        lm.tare_position();
        lb.tare_position();
        rf.tare_position();
        rm.tare_position();
        rb.tare_position();
        
        // Reset position variables
        x = 0;
        y = 0;
        theta = 0;
    }

private:
    pros::Motor lf; // Left front motor
    pros::Motor lm; // Left middle motor
    pros::Motor lb; // Left back motor
    pros::Motor rf; // Right front motor
    pros::Motor rm; // Right middle motor
    pros::Motor rb; // Right back motor
    pros::IMU imu_sensor; // IMU sensor

    // Wheel properties
    double wheel_diameter = 69.85; // mm
    double wheel_circumference = M_PI * wheel_diameter; // mm
    double encoder_counts_per_revolution = 360.0; // Encoder counts per full wheel rotation
    double PosConvert = wheel_circumference / encoder_counts_per_revolution; // mm per encoder count

    // Position variables
    double x;    // Robot's x position in mm
    double y;    // Robot's y position in mm
    double theta; // Robot's heading in radians
};

// Define the autonomous function
void autonomous() {
    tankBase robot; // Initialize the Robot
    std::vector<std::pair<double, double>> waypoints = {
        {0, 0}, {500, 500}, {1000, 1000}, {1500, 500}, {2000, 0} // Example waypoints in mm
    };
    double lookahead_distance = 500; // Lookahead distance in mm

    // Create a PurePursuit instance
    PurePursuit purePursuit(waypoints, lookahead_distance);

    // Reset odometry at the start of autonomous
    robot.reset_odometry();

    // Main loop for autonomous
    while (true) {
        // Update odometry based on motor encoder readings
        robot.update_odometry();

        // Get current robot position and heading
        double x = robot.get_x();
        double y = robot.get_y();
        double heading = robot.get_heading();

        // Calculate the steering angle using Pure Pursuit
        double steering_angle = purePursuit.calculate_steering_angle(x, y, heading);

        // Control motors based on the steering angle (simplified logic)
        double speed = 80; // Example forward speed
        double turn_factor = steering_angle * 10; // Adjust the multiplier as necessary
        lf.move(speed + turn_factor);
        lm.move(speed + turn_factor);
        lb.move(speed + turn_factor);
        rf.move(speed - turn_factor);
        rm.move(speed - turn_factor);
        rb.move(speed - turn_factor);

        pros::delay(20); // Delay for control loop

        if (x >= 2000 && y >= 0) { //end the loop
            break;
        }
    }
}

void initialize() {
	pros::lcd::initialize();
	lf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	rf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	intakeLower.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intakeUpper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	imu_sensor.reset(true);
    imu_sensor.set_data_rate(5);

	pros::Task serial_read(serialRead);

	master.clear();
}

void opcontrol() {
	while (true){
		leftY = bound_value(master.get_analog(ANALOG_LEFT_Y)*SCALING_FACTOR);
		rightX = bound_value(master.get_analog(ANALOG_RIGHT_X)*SCALING_FACTOR);

        lf.move_velocity(leftY - rightX);
        lm.move_velocity(leftY - rightX);
        lb.move_velocity(leftY - rightX);

        rf.move_velocity(leftY + rightX);
        rm.move_velocity(leftY + rightX);
        rb.move_velocity(leftY + rightX);

		if(master.get_digital(DIGITAL_R1)){
			intakeLower.move(110);
			intakeUpper.move(110);
		}
        else if(master.get_digital(DIGITAL_L1)){
			intakeLower.move(-110);
			intakeUpper.move(-110);
		}
		else{
			intakeLower.move(0);
			intakeUpper.move(0);
		}

        if(master.get_digital(DIGITAL_R2)){
            conveyor.move(110);
        }
        else if(master.get_digital(DIGITAL_L2)){
            conveyor.move(-110);
        }
        else{
            conveyor.move(0);
        }

        if(master.get_digital_new_press(DIGITAL_A)) actuated = !actuated;

        if(actuated) solenoid.set_value(0);
        else solenoid.set_value(1);

		pros::delay(5);
	}
}
