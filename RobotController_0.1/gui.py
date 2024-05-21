from tkinter import *


class Light(Canvas):
    """
    Class to construct an oval element in the GUI
    """
    def __init__(self, master, **kwargs):
        """
        initializer
        :param master: the root of the tkinter menu
        :param kwargs: possible arguments for the canvas.
        """
        Canvas.__init__(self, master, width=50, height=50, **kwargs)
        self.oval_id = self.create_oval(2, 2, 50, 50, fill='gray')

    def turn_on(self):
        """
        Changes the color of the oval to green
        """
        self.itemconfig(self.oval_id, fill='green')

    def turn_off(self):
        """
        Changes the color of the oval to gray
        """
        self.itemconfig(self.oval_id, fill='gray')

    def turn_ready(self):
        """
        Changes the color of the oval to red
        """
        self.itemconfig(self.oval_id, fill='yellow')


class GUI:
    """
    Main class for the GUI
    """
    def __init__(self):
        """
        Initializer for tkinter menu
        """
        self.root = Tk()
        self.pressed_buttons = set()

        self.mode = None
        self.running = False

        self.button_START = Button(self.root, text="START", padx=150, pady=50, state=DISABLED, command=self.start_pressed, fg="black",
                           bg="green")
        self.button_STOP = Button(self.root, text="STOP", padx=140, pady=50, state=NORMAL, command=self.stop_pressed_init, fg="black",
                           bg="red")
        self.button_SELECT_ROBOT = Button(self.root, text="Execute program on robot", padx=150, pady=50, state=NORMAL, command=self.robot_pressed, fg="white",
                           bg="black")
        self.button_SELECT_SIMULATION = Button(self.root, text="Execute program in simulation", padx=150, pady=50, state=NORMAL, command=self.simulation_pressed,
                           fg="white", bg="black")
        self.button_SELECT_BOTH = Button(self.root, text="Execute program on robot and in simulation", padx=100, pady=50, state=NORMAL, command=self.both_pressed,
                           fg="white", bg="black")

        self.button_START.grid(row=0, column=0, padx=10, pady=10)
        self.button_STOP.grid(row=0, column=2, padx=10, pady=10)
        self.button_SELECT_ROBOT.grid(row=1, column=0, padx=10, pady=10)
        self.button_SELECT_SIMULATION.grid(row=1, column=1, padx=10, pady=10)
        self.button_SELECT_BOTH.grid(row=1, column=2, padx=10, pady=10)

        self.light_SELECT_ROBOT = Light(self.root)
        self.light_SELECT_ROBOT.grid(row=2, column=0, padx=10, pady=10)

        self.light_SELECT_SIMULATION = Light(self.root)
        self.light_SELECT_SIMULATION.grid(row=2, column=1, padx=10, pady=10)

        self.light_SELECT_BOTH = Light(self.root)
        self.light_SELECT_BOTH.grid(row=2, column=2, padx=10, pady=10)

        self.title = Label(self.root, text="Patch test preparer")
        self.title.grid(row=0, column=1)

    def start_pressed(self):
        """
        Button method for when clicking on the start button
        """
        self.button_START["state"] = NORMAL
        self.button_STOP["state"] = NORMAL
        self.button_SELECT_ROBOT["state"] = DISABLED
        self.button_SELECT_SIMULATION["state"] = DISABLED
        self.button_SELECT_BOTH["state"] = DISABLED

        if "Robot" in self.pressed_buttons:
            self.light_SELECT_ROBOT.turn_on()
            self.mode = "robot"
        else:
            self.light_SELECT_ROBOT.turn_off()

        if "Simulation" in self.pressed_buttons:
            self.light_SELECT_SIMULATION.turn_on()
            self.mode = "simulation"
        else:
            self.light_SELECT_SIMULATION.turn_off()

        if "Robot and Simulation" in self.pressed_buttons:
            self.light_SELECT_BOTH.turn_on()
            self.mode = "both"
        else:
            self.light_SELECT_BOTH.turn_off()

        self.running = True

    def stop_pressed_init(self):
        """
        Button method for when clicking on the stop button
        """
        self.mode = None

        self.button_START["state"] = DISABLED
        self.button_STOP["state"] = DISABLED
        self.button_SELECT_ROBOT["state"] = DISABLED
        self.button_SELECT_SIMULATION["state"] = DISABLED
        self.button_SELECT_BOTH["state"] = DISABLED

        self.light_SELECT_ROBOT.turn_off()
        self.light_SELECT_SIMULATION.turn_off()
        self.light_SELECT_BOTH.turn_off()

        self.pressed_buttons.clear()

        if self.running:
            self.running = False
        else:
            self.stop_pressed()

    def stop_pressed(self):
        """
        Button method for when the program has run the stop command and the buttons should be clickable again.
        """
        self.button_START["state"] = DISABLED
        self.button_STOP["state"] = NORMAL
        self.button_SELECT_ROBOT["state"] = NORMAL
        self.button_SELECT_SIMULATION["state"] = NORMAL
        self.button_SELECT_BOTH["state"] = NORMAL

    def robot_pressed(self):
        """
        Button method for when clicking on the 'Execute program on robot'
        """
        self.pressed_buttons.add("Robot")

        self.button_START["state"] = NORMAL
        self.button_STOP["state"] = NORMAL
        self.button_SELECT_ROBOT["state"] = NORMAL
        self.button_SELECT_SIMULATION["state"] = DISABLED
        self.button_SELECT_BOTH["state"] = DISABLED

        self.light_SELECT_ROBOT.turn_ready()
        self.light_SELECT_SIMULATION.turn_off()
        self.light_SELECT_BOTH.turn_off()

    def simulation_pressed(self):
        """
        Button method for when clicking on the 'Execute program in simulation'
        """
        self.pressed_buttons.add("Simulation")

        self.button_START["state"] = NORMAL
        self.button_STOP["state"] = NORMAL
        self.button_SELECT_ROBOT["state"] = DISABLED
        self.button_SELECT_SIMULATION["state"] = NORMAL
        self.button_SELECT_BOTH["state"] = DISABLED

        self.light_SELECT_ROBOT.turn_off()
        self.light_SELECT_SIMULATION.turn_ready()
        self.light_SELECT_BOTH.turn_off()

    def both_pressed(self):
        """
        Button method for when clicking on the 'Execute program on robot and in simulation'
        """
        self.pressed_buttons.add("Robot and Simulation")

        self.button_START["state"] = NORMAL
        self.button_STOP["state"] = NORMAL
        self.button_SELECT_ROBOT["state"] = DISABLED
        self.button_SELECT_SIMULATION["state"] = DISABLED
        self.button_SELECT_BOTH["state"] = NORMAL

        self.light_SELECT_ROBOT.turn_off()
        self.light_SELECT_SIMULATION.turn_off()
        self.light_SELECT_BOTH.turn_ready()

    def run(self):
        """
        Runs the program
        """
        self.root.mainloop()


if __name__ == "__main__":
    test = GUI()
    test.run()
