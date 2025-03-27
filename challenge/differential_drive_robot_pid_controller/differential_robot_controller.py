# Imported libraries
import math # Used for mathematical calculations
import pygame # Used for creating the graphical simulation environment

class Environment:
    """
        Handles the visualization and rendering of the environment.
    """
    def __init__(self, dimensions):

        """ 
            Sets up the PyGame display window
            Defines color constants
            Creates a font for displaying information
            Initializes a trail tracking list
        """

        pygame.init()
        self.width, self.height = dimensions
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)

        pygame.display.set_caption("Differential Drive Robot Simulation")
        self.map = pygame.display.set_mode((self.width, self.height))
        
        self.font = pygame.font.Font('freesansbold.ttf', 30)
        
        self.trail_set = []

    """
        Displays robot information (velocities, orientation, position)
    """
    def write_info(self, screen, vl, vr, phi, x, y):

        txt = f"Vl = {vl:.2f} m/s | Vr = {vr:.2f} m/s | φ = {int(math.degrees(phi))}° | x = {x:.2f} m | y = {y:.2f} m"
        text = self.font.render(txt, True, self.white, self.black)
        text_rect = text.get_rect(center=(self.width // 2, 30))
        screen.blit(text, text_rect)

    """
        Draws the robot's local coordinate axes
    """
    def draw_robot_frame(self, screen, pos, rotation, m2p):

        n = 80
        centerx = pos[0] * m2p
        centery = pos[1] * m2p
        x_axis = (centerx + n*math.cos(-rotation), centery + n*math.sin(-rotation))
        y_axis = (centerx + n*math.cos(-rotation+math.pi/2), centery + n*math.sin(-rotation+math.pi/2))
        pygame.draw.line(screen, self.red, (centerx, centery), x_axis, 3)
        pygame.draw.line(screen, self.green, (centerx, centery), y_axis, 3)

    """
        Tracks and draws the robot's path
    """
    def draw_trail(self, screen, pos, m2p):

        pos_px = (pos[0] * m2p, pos[1] * m2p)
        self.trail_set.append(pos_px)
        
        if len(self.trail_set) > 2:
            pygame.draw.lines(screen, self.yellow, False, self.trail_set, 2)
        
        if len(self.trail_set) > 1000:
            self.trail_set.pop(0)

    def draw_startpoint(self, screen, pos, m2p, color=(0, 0, 255), radius=10):

        x_pix = pos[0] * m2p
        y_pix = pos[1] * m2p
        pygame.draw.circle(screen, color, (int(x_pix), int(y_pix)), radius)
    
    """
        Marks the goal / destination point
    """
    def draw_setpoint(self, screen, pos, m2p, color=(255, 0, 0), radius=10):

        x_pix = pos[0] * m2p
        y_pix = pos[1] * m2p
        pygame.draw.circle(screen, color, (int(x_pix), int(y_pix)), radius)
    
    """
        Shows a temporary message on the screen
    """
    def display_message(self, screen, message, duration=2000):

        text = self.font.render(message, True, (0, 255, 0), (0, 0, 0))
        text_rect = text.get_rect(center=(self.width // 2, self.height // 2))

        screen.blit(text, text_rect)
        pygame.display.update()

        pygame.time.wait(duration)

"""
    Represents the robot with its physical and control characteristics
"""
class DifferentialRobot:

    def __init__(self, startpos, orientation, desiredpos, robot_image, half_width, dt):
    
        """
            Sets initial position, orientation
            Defines robot geometry (width)
            Configures velocity parameters
            Sets PID (Proportional-Integral-Derivative) control gains
            Loads and prepares robot image
        """

        self.m2p = 3779.52  # Meters to pixels conversion
        
        self.b = half_width  # Half-width of robot
        
        # Initial state
        self.x = startpos[0] # x-component of start position to be defined
        self.y = startpos[1] # y-component of start position to be defined
        self.phi = orientation # start orientation to be defined
        
        self.vl = 0.01  # Initial left wheel velocity
        self.vr = 0.01  # Initial right wheel velocity
        # PID controller control inputs:
        self.V = 0  # Linear velocity
        self.omega = 0  # Angular velocity
        
        # Speed limits
        self.maxspeed = 0.02
        self.minspeed = -0.02
        
        self.dt = dt # Time step to be defined
        
        # Setpoint (goal position)
        self.xc_d = desiredpos[0] # Desired x position
        self.yc_d = desiredpos[1]  # Desired y position
        
        # PID gains
        self.Kp_V = 40.0  # Proportional gain for linear velocity
        self.Ki_V = 0.1  # Integral gain for linear velocity
        self.Kd_V = 0.05  # Derivative gain for linear velocity
        self.Kp_w = 40.0  # Proportional gain for angular velocity
        self.Ki_w = 0.1  # Integral gain for angular velocity
        self.Kd_w = 0.05  # Derivative gain for angular velocity
        
        # Error tracking
        self.int_e_x_c_local = 0
        self.int_e_y_c_local = 0
        self.int_e_phi = 0
        self.prev_e_x_c_local = 0
        self.prev_e_phi = 0
        
        # Graphics
        self.img = pygame.image.load(robot_image)
        self.img = pygame.transform.rotate(self.img, 90)
        self.img = pygame.transform.scale(self.img, 
                                          (self.img.get_width() // 4, 
                                           self.img.get_height() // 4))
        self.rotated = self.img
        self.rect = self.rotated.get_rect()

    def update_pose(self):
        # Threshold for reaching the goal
        position_threshold = 0.001
        angular_threshold = 0.05

        # Calculate errors
        e_x_c = self.xc_d - self.x
        e_y_c = self.yc_d - self.y
        setpoint_distance = math.hypot(e_x_c, e_y_c)
        phi_d = math.atan2(e_y_c, e_x_c)
        e_phi = wrap_angle(phi_d - self.phi)

        # Stop if close to goal
        if setpoint_distance < position_threshold and abs(e_phi) < angular_threshold:
            self.V = 0
            self.omega = 0
            return False
        
        # Update error integrals
        self.int_e_x_c_local += e_x_c * self.dt
        self.int_e_phi += e_phi * self.dt

        # Calculate error derivatives
        der_e_x_c_local = (e_x_c - self.prev_e_x_c_local) / self.dt
        der_e_phi = (e_phi - self.prev_e_phi) / self.dt

        # PID control for velocity
        self.V = self.Kp_V * e_x_c + self.Ki_V * self.int_e_x_c_local + self.Kd_V * der_e_x_c_local
        self.omega = self.Kp_w * e_phi + self.Ki_w * self.int_e_phi + self.Kd_w * der_e_phi

        # Limit velocities
        self.V = max(min(self.V, self.maxspeed), self.minspeed)
        self.omega = max(min(self.omega, 2.0), -2.0)

        # Calculate wheel velocities
        self.vr = self.V + ((2*self.b) / 2) * self.omega
        self.vl = self.V - ((2*self.b) / 2) * self.omega

        # Update pose
        self.x += self.V * math.cos(self.phi) * self.dt
        self.y += self.V * math.sin(self.phi) * self.dt
        self.phi += self.omega * self.dt

        # Update previous errors
        self.prev_e_x_c_local = e_x_c
        self.prev_e_phi = e_phi

        return True

def wrap_angle(angle):
    # Normalize angle to [-π, π] range
    return (angle + math.pi) % (2 * math.pi) - math.pi

def parse_coordinate_input(prompt, conversion_factor=3779.52):
    """
    Parse user input into a list of two float numbers with pixel-meter conversion
    
    Args:
        prompt (str): Prompt message for user input
        conversion_factor (float): Pixels per meter conversion factor
    
    Returns:
        tuple: Coordinates converted to meters
    """
    while True:
        try:
            # Prompt user and get input
            print(prompt)
            print("Enter two numbers separated by a space (e.g., '0 0' or '500 750')")
            
            # Get input and strip whitespace
            user_input = input("> ").strip()
            
            # Split input and remove any empty strings
            coords_input = [x for x in user_input.split() if x]
            
            # Validate input
            if len(coords_input) != 2:
                print("Error: Please enter exactly two numbers.")
                continue
            
            # Convert to floats
            coords_pixels = [float(coord) for coord in coords_input]
            
            # Convert pixels to meters
            coords_meters = (
                coords_pixels[0] / conversion_factor, 
                coords_pixels[1] / conversion_factor
            )
            
            print(f"Converted coordinates: {coords_meters[0]:.4f} m, {coords_meters[1]:.4f} m")
            
            return coords_meters
        
        except ValueError:
            print("Error: Invalid input. Please enter numeric values.")

def main():
    # Simulation settings
    dims = (1920, 1080)
    dt = 0.01
    conversion_factor=3779.52

    # Get user inputs
    print("=== Differential Drive Robot Simulation === ")

    # Get start position input
    startpos = parse_coordinate_input("Enter start position coordinates", conversion_factor)

    # Get orientation
    while True:
        try:
            orientation = float(input("\nEnter initial orientation (in radians): "))
            break
        except ValueError:
            print("Error: Please enter a valid number for orientation.")

    # Get desired position input
    desiredpos = parse_coordinate_input("Enter desired position coordinates", conversion_factor=3779.52)
    
    # Robot image path
    robot_image=r"/home/chrisrvt/projects/Implementation_of_Intelligent_Robotics/challenge/differential_drive_robot_pid_controller/difdrive.png"  # Replace with your robot image path
    
    # Half-width of robot
    half_width = (0.001)/2

    # Initialize environment and robot
    environment = Environment(dims)
    robot = DifferentialRobot(
        startpos=startpos,
        orientation=orientation,
        desiredpos=desiredpos,
        robot_image=robot_image,
        half_width=half_width*conversion_factor,
        dt=dt
    )

    # Main simulation loop
    running = True
    clock = pygame.time.Clock()
    iteration_count = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update robot pose
        is_moving = robot.update_pose()

        # Drawing
        environment.map.fill(environment.white)
        environment.write_info(environment.map, robot.vl, robot.vr, robot.phi, robot.x, robot.y)
        
        # Draw robot frame and trail
        x_pix = robot.x * robot.m2p
        y_pix = robot.y * robot.m2p
        robot.rotated = pygame.transform.rotozoom(robot.img, math.degrees(robot.phi), 1)
        robot.rect = robot.rotated.get_rect(center=(x_pix, y_pix))

        # Draw start point
        environment.draw_startpoint(
            screen=environment.map,
            pos=(robot.x, robot.y),
            m2p=robot.m2p
        )

        # Draw setpoint
        environment.draw_setpoint(
            screen=environment.map,
            pos=(robot.xc_d, robot.yc_d),
            m2p=robot.m2p
        )
        
        environment.map.blit(robot.rotated, robot.rect)
        environment.draw_robot_frame(environment.map, (robot.x, robot.y), robot.phi, robot.m2p)
        environment.draw_trail(environment.map, (robot.x, robot.y), robot.m2p)

        pygame.display.update()
        clock.tick(60)  # Limit to 60 FPS

        # Exit if robot reaches goal
        if not is_moving:
            environment.display_message(
                screen=environment.map,
                message='Goal Reached!',
                duration=2000
            )
            running = False

    pygame.quit()

if __name__ == "__main__":
    main()