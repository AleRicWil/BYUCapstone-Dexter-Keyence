import customtkinter as ctk
from tkinter import messagebox
import pandas as pd
import numpy as np
import time
import os
import json
import csv
import threading
import queue
from datetime import date, datetime
from reportlab.pdfgen import canvas
import webbrowser
import Measure_Hub as MH  
import Measure_Arm as MA
import Perform_Scan_LJS640 as PS

# Configure CustomTkinter appearance
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class Dexter_Capstone_UI:
    
    # Setup
    def __init__(self, master, q):
        """Initialize the UI with window settings and file paths."""
        self.master = master
        self.master.title("Dexter-Capstone 2025")
        self.master.geometry("1200x800")
        self.default_button_size = 300
        self.axle_database_path = r"C:\Users\Public\CapstoneUI\Axle_Database.csv"
        self.arm_database_path = r"C:\Users\Public\CapstoneUI\Arm_Database.csv"
        self.calibration_path = r"C:\Users\Public\CapstoneUI\Calibration History.csv"
        self.temp_scan_pathA = r'C:\Users\Public\CapstoneUI\temporary_scan.csv'
        self.json_path = r"C:\Users\Public\CapstoneUI\arm_types.json"
        self.arm_types = {}
        self.load_arm_types()
        self.selected_arm_type = None
        self.get_hub_calibration()

        self.debug_flag = True
        self.test_num = '00'
        
        # Create a persistent frame for the message log at the bottom
        self.log_frame = ctk.CTkFrame(self.master)
        self.log_frame.pack(side=ctk.BOTTOM, fill=ctk.X, padx=20, pady=(0, 20))
        self.message_log = ctk.CTkTextbox(self.log_frame, height=150, state='disabled')
        self.message_log.pack(fill=ctk.BOTH, expand=True)
        
        # Create a main content frame above the log
        self.content_frame = ctk.CTkFrame(self.master)
        self.content_frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=(20, 0))
        
        self.open_home_screen()

    def load_arm_types(self):
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, 'r') as f:
                    loaded = json.load(f)
                    self.arm_types = {}
                    for k, v in loaded.items():
                        self.arm_types[k] = v
            except:
                print('No valid arm types in .json file')
                self.arm_types = {}
        else:
            self.arm_types = {}

    def get_default_rotations(self):
        return {
            'left': [{'axis': 'z', 'angle': 0}, {'axis': 'x', 'angle': 270}],
            'right': [{'axis': 'z', 'angle': 180}, {'axis': 'x', 'angle': 180}]
        }

    def parse_rotations(self, s):
        rots = []
        for p in s.split(','):
            p = p.strip()
            if p:
                axis, angle_str = p.split(':')
                rots.append({'axis': axis.strip(), 'angle': float(angle_str.strip())})
        return rots

    def save_arm_types(self):
        os.makedirs(os.path.dirname(self.json_path), exist_ok=True)
        with open(self.json_path, 'w') as f:
            json.dump(self.arm_types, f, indent=4)

    def clear_window(self):
        """Remove all widgets from the content frame and unbind the Return key."""
        self.master.unbind("<Return>")
        for widget in self.content_frame.winfo_children():
            widget.destroy()

    def setup_screen(self, title_text, content_callback, home_button=True):
        """Set up a screen with a title, content, and optional home button, preserving the message log."""
        self.clear_window()
        
        # Add title
        title = ctk.CTkLabel(self.content_frame, text=title_text, font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))
        
        # Call the content callback to add specific content
        content_callback(self.content_frame)
        
        # Add home button if requested
        if home_button:
            bottom_frame = ctk.CTkFrame(self.content_frame)
            bottom_frame.pack(side=ctk.BOTTOM, fill=ctk.X, pady=(20, 0))
            ctk.CTkButton(master=bottom_frame, text="Return to Main Menu", command=self.open_home_screen, width=self.default_button_size).pack()

    def log_message(self, message):
        """Append a message to the log and scroll to the bottom."""
        def update_log():
            if hasattr(self, 'message_log') and self.message_log:
                self.message_log.configure(state='normal')
                self.message_log.insert(ctk.END, message + '\n')
                self.message_log.configure(state='disabled')
                self.message_log.see(ctk.END)
            else:
                print(f"Fallback: {message}")
        self.master.after(0, update_log)

    def update_status(self, message):
        """Update the log with the given message (replaces old status label functionality)."""
        self.log_message(message)

    def get_input(self, message, options=["Yes", "No"]):
        """Show a prompt with buttons and return the user's choice."""
        self.user_response = None
        self.response_event = threading.Event()

        def content(frame):
            ctk.CTkLabel(frame, text=message, font=ctk.CTkFont(size=18), wraplength=1000).pack(pady=(20, 20))
            for option in options:
                ctk.CTkButton(frame, text=option, command=lambda opt=option: self.set_response(opt)).pack(pady=(10, 0))

        self.setup_screen("Manual Interaction", content, home_button=False)
        self.response_event.wait()  # Block until user responds
        return self.user_response

    def set_response(self, response):
        """Set the user's response and signal completion."""
        self.user_response = response
        self.response_event.set()


    # General
    def run_scanner(self):
        if self.scan_type != 'real':
            messagebox.showerror("Error", "Scanning is only available for 'real' scan types.")
            return
        def content(frame):
            ctk.CTkLabel(frame, text="Scanning...", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
        if self.type == 'hub':
            self.setup_screen("TorFlex Axle — Measure Hub Alignment", content, home_button=False)
        elif self.type == 'arm':
            self.setup_screen("TorFlex Axle — Measure Arm Alignment", content, home_button=False)
        self.master.update()

        data = PS.perform_scan().astype(float)
        for i in data:
            i = (i - 2**15) * .0102

        # if self.type == 'arm':
        #     self.temp_scan_pathA = fr'C:\Users\Public\CapstoneUI\TempScans\{self.arm_id}\{self.selected_arm_type}_{datetime.now().strftime("%Y-%m-%d_%H%M%S")}.csv'
        # np.savetxt(self.temp_scan_pathA, data, delimiter=',', header='X Y Z')
        
        # new block to save multiple scans of the same arm ID, adding user-provided test_num to filename
        if self.type == 'arm':
            self.test_num = self.test_num_combo.get() if hasattr(self, 'test_num_combo') else ''
            if self.test_num and self.test_num != 'None: 00':  # Optional: add more validation, e.g., if not test_num.isdigit(): messagebox.showerror("Error", "Test # must be a number."); return
                filename = f"{self.selected_arm_type}_{self.test_num}_{datetime.now().strftime('%Y-%m-%d_%H%M%S')}.csv"
            else:
                filename = f"{self.selected_arm_type}_00_{datetime.now().strftime('%Y-%m-%d_%H%M%S')}.csv"
            
            self.temp_scan_pathA = fr'C:\Users\Public\CapstoneUI\TempScans\{self.arm_id}\{filename}'
            os.makedirs(os.path.dirname(self.temp_scan_pathA), exist_ok=True)  # Ensures the directory exists before saving

        if self.type == 'hub':
            self.hub_scan_fileA = self.temp_scan_pathA
            self.calc_hub_alignment()
        elif self.type =='arm':
            self.arm_scan_fileA = self.temp_scan_pathA
            self.calc_arm_alignment()

    def run_repeated_scanner(self):
        def content(frame):
            ctk.CTkLabel(frame, text=f"Scanning...", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
        if self.type == 'hub':
            self.setup_screen("TorFlex Axle — Measure Hub Alignment", content, home_button=False)
        elif self.type == 'arm':
            self.setup_screen("TorFlex Axle — Measure Arm Alignment", content, home_button=False)
        self.master.update()

        self.toe_arr = np.array([])
        self.camber_arr = np.array([])
        self.total_angle_arr = np.array([])

        for index in range(self.scan_count):
            data = PS.perform_scan().astype(float)
            for i in data:
                i = (i - 2**15) * .0102

            if index + 1 < 10:
                scan_text = f'{self.arm_id}0{index + 1}'
            else:
                scan_text = f'{self.arm_id}{index + 1}'

            os.makedirs(os.path.dirname(self.arm_database_path), exist_ok=True)
            self.initialize_csv(self.arm_database_path, ["Arm ID", "Bar Toe", "Bar Camber",
                                                          "Spindle Toe", "Spindle Camber",
                                                          "Toe", "Camber",
                                                            "Total Relative Angle", "Date Scanned"])
            df = pd.read_csv(self.arm_database_path, dtype=str)
            if scan_text not in df["Arm ID"].values:
                pd.concat([df, pd.DataFrame([{"Arm ID": scan_text}])], ignore_index=True).to_csv(self.arm_database_path, index=False)
                self.update_status(f"Arm ID {scan_text} added to database.")
            else:
                self.update_status(f"Arm ID {scan_text} already exists.")

            if self.type == 'arm':
                self.temp_scan_pathA = fr'C:\Users\Public\CapstoneUI\TempScans\{scan_text}.csv'
            np.savetxt(self.temp_scan_pathA, data, delimiter=',', header='X Y Z')
        
        for self.index in range(self.scan_count):
            if self.index + 1 < 10:
                scan_text = f'{self.arm_id}0{self.index + 1}'
            else:
                scan_text = f'{self.arm_id}{self.index + 1}' 

            self.scan_type = 'real'
            if self.type == 'hub':
                self.hub_scan_fileA = self.temp_scan_pathA
                self.calc_hub_alignment()
            elif self.type =='arm':
                self.temp_scan_pathA = fr'C:\Users\Public\CapstoneUI\TempScans\{scan_text}.csv'
                self.arm_scan_fileA = self.temp_scan_pathA
                self.calc_repeated_arm_alignment(scan_text=scan_text)
    
    def validate_file_and_start(self):
        scan_file = self.existing_scan_entry.get().strip()
        if not scan_file or not os.path.isfile(scan_file):
            messagebox.showerror("Error", "Please enter a valid and accessible file path.")
            return
        if self.type == 'hub':
            self.hub_scan_fileA = scan_file
            self.calc_hub_alignment()
        elif self.type =='arm':
            self.arm_scan_fileA = scan_file
            test_num = self.test_num_combo.get() if hasattr(self, 'test_num_combo') else '00'
            if test_num == 'None: 00':
                test_num = '00'
            self.test_num = test_num
            self.calc_arm_alignment()

    def validate_number(self):
        scan_count = self.num_scans.get().strip()
        if not scan_count.isnumeric():
            messagebox.showerror("Error", "Please enter a valid number of scans.")
            return
        else:
            self.scan_count = int(scan_count)
            self.run_repeated_scanner()


    # Home screen
    def open_home_screen(self):
        """Display the home screen with navigation options."""
        def content(frame):
            ctk.CTkButton(frame, text="Measure Hub", command=self.measure_hub, width=self.default_button_size).pack(pady=(0, 20))
            ctk.CTkButton(frame, text="Measure Crank Arm", command=self.measure_arm, width=self.default_button_size).pack(pady=(0, 20))
            ctk.CTkButton(frame, text="Hub Measurement Calibration", command=self.calibrate_hub, width=self.default_button_size).pack(pady=(0, 20))
            ctk.CTkButton(frame, text="Crank Arm Measurement Calibration", command=self.calibrate_arm, width=self.default_button_size).pack(pady=(0, 20))

        self.setup_screen("Dexter TorFlex Axle — Hub and Crank Arm Alignment Software", content, home_button=False)

    def measure_hub(self):
        self.type = 'hub'
        """Show screen to enter axle ID for hub measurement."""
        def save_axleID():
            self.axle_id = self.barcode_entry.get().strip()
            if not self.axle_id:
                messagebox.showerror("Error", "Please enter an Axle ID")
                return

            os.makedirs(os.path.dirname(self.axle_database_path), exist_ok=True)
            self.initialize_csv(self.axle_database_path, ["Axle ID", "Left Toe", "Left Camber", "Right Toe", "Right Camber", "Total Toe", "Date Scanned"])
            df = pd.read_csv(self.axle_database_path, dtype=str)
            if self.axle_id not in df["Axle ID"].values:
                pd.concat([df, pd.DataFrame([{"Axle ID": self.axle_id}])], ignore_index=True).to_csv(self.axle_database_path, index=False)
                self.update_status(f"Axle ID {self.axle_id} added to database.")
            else:
                self.update_status(f"Axle ID {self.axle_id} already exists.")

            self.show_hub_scan_screen()

        def content(frame):
            self.barcode_entry = ctk.CTkEntry(frame, placeholder_text="Enter Axle Identifier", width=self.default_button_size)
            self.barcode_entry.pack(pady=(0, 20))
            ctk.CTkButton(frame, text="Enter", command=save_axleID, width=self.default_button_size).pack(pady=(0, 20))
            self.master.bind("<Return>", lambda event: save_axleID())

        self.setup_screen("TorFlex Axle — Measure Hub Alignment", content)

    def measure_arm(self):
        self.type = 'arm'
        """Show screen to enter arm assembly ID for arm measurement."""
        def save_armID():
            self.arm_id = self.barcode_entry.get().strip()
            if not self.arm_id:
                messagebox.showerror("Error", "Please enter an Arm ID")
                return

            os.makedirs(os.path.dirname(self.arm_database_path), exist_ok=True)
            self.initialize_csv(self.arm_database_path, ["Arm ID", "Test #", "Arm Type", "Toe (deg)", "Camber (deg)",
                                                    "Bar Toe (deg)", "Bar Camber (deg)", "Spindle Toe (deg)", "Spindle Camber (deg)",
                                                    "Total Relative Angle (deg)", "Date Scanned (yyyy-mm-dd_hhmmss)"])
            df = pd.read_csv(self.arm_database_path, dtype=str)
            if self.arm_id not in df["Arm ID"].values:
                pd.concat([df, pd.DataFrame([{"Arm ID": self.arm_id}])], ignore_index=True).to_csv(self.arm_database_path, index=False)
                self.update_status(f"Arm ID {self.arm_id} added to database.")
            else:
                self.update_status(f"Arm ID {self.arm_id} already exists. Will ask to overwrite after measurement.")

            self.show_arm_type_selection()

        def content(frame):
            self.barcode_entry = ctk.CTkEntry(frame, placeholder_text="Enter Arm Identifier", width=self.default_button_size)
            self.barcode_entry.pack(pady=(0, 20))
            ctk.CTkButton(frame, text="Enter", command=save_armID, width=self.default_button_size).pack(pady=(0, 20))
            self.master.bind("<Return>", lambda event: save_armID())

        self.setup_screen("TorFlex Axle — Measure Crank Arm Alignment", content)

    def calibrate_hub(self):
        def content(frame):
            ctk.CTkLabel(frame, text='Current offsets:', font=ctk.CTkFont(size=20, weight='bold')).pack(pady=(20, 40))
            current_offsets = f"Left X offset: {self.calibrationL['x']}°\nLeft Y offset: {self.calibrationL['y']}°\nRight X offset: {self.calibrationR['x']}°\nRight Y offset: {self.calibrationR['y']}°"
            ctk.CTkLabel(frame, text=current_offsets, font=ctk.CTkFont(size=18)).pack(pady=(10, 20))
            ctk.CTkButton(frame, text='Perform new calibration', command=self.input_calibration_axle_data, width=200).pack(pady=(40, 0))
            ctk.CTkButton(frame, text='Reset calibration to zero', command=self.reset_calibration, width=200).pack(pady=(40, 0))
        self.get_hub_calibration()
        self.setup_screen("Hub Alignment Calibration", content)

    def calibrate_arm(self):
        def content(frame):
            ctk.CTkLabel(frame, text="Crank Arm Calibration - Coming Soon", font=ctk.CTkFont(size=18)).pack(pady=(0, 20))
        self.setup_screen("TorFlex Axle — Calibrate Crank Arm", content)


    # Measure hub
    def get_hub_calibration(self):
        os.makedirs(os.path.dirname(self.calibration_path), exist_ok=True)
        self.initialize_csv(self.calibration_path, ["Left Rotation About X", "Left Rotation About Y", "Right Rotation About X", "Right Rotation About Y", "Date"])
        df = pd.read_csv(self.calibration_path)
        if df.empty:
            self.calibrationL = self.calibrationR = {"x": 0, "y": 0}
            self.calibration_date = date.today()
            pd.DataFrame([{"Left Rotation About X": 0, "Left Rotation About Y": 0, "Right Rotation About X": 0, "Right Rotation About Y": 0, "Date": self.calibration_date}]).to_csv(self.calibration_path, index=False)
        else:
            last = df.iloc[-1]
            self.calibration_date = last["Date"]
            self.calibrationL = {"x": float(last["Left Rotation About X"]), "y": float(last["Left Rotation About Y"])}
            self.calibrationR = {"x": float(last["Right Rotation About X"]), "y": float(last["Right Rotation About Y"])}

    def show_hub_scan_screen(self):
        def content(frame):
            ctk.CTkLabel(frame, text=f"Axle ID: {self.axle_id}", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
            ctk.CTkButton(frame, text="Start Scanner", command=self.run_scanner, width=200).pack(pady=(40, 0))
            scan_frame = ctk.CTkFrame(frame)
            scan_frame.pack(pady=(40, 0))
            ctk.CTkButton(scan_frame, text="Measure from existing scan:", command=self.validate_file_and_start, width=200).pack(side=ctk.LEFT, padx=(0, 10))
            self.existing_scan_entry = ctk.CTkEntry(scan_frame, placeholder_text="enter scan file path", width=300)
            self.existing_scan_entry.pack(side=ctk.LEFT)
            mode_frame = ctk.CTkFrame(frame)
            mode_frame.pack(pady=(20, 0))
            ctk.CTkLabel(mode_frame, text="Manual Mode:", font=ctk.CTkFont(size=18)).pack(side=ctk.LEFT, padx=(0, 10))
            self.auto_mode_switch = ctk.CTkSwitch(mode_frame, text="Auto/Manual", command=self.update_auto_mode)
            self.auto_mode_switch.pack(side=ctk.LEFT)
            self.auto_flag = self.auto_mode_switch.get() == 0
            ctk.CTkButton(frame, text="Back", command=self.measure_hub, width=200).pack(pady=(40, 0))
            self.master.bind("<Return>", lambda event: self.run_scanner())
        self.setup_screen("TorFlex Axle — Measure Hub Alignment", content)

    def update_auto_mode(self):
        self.auto_flag = self.auto_mode_switch.get() == 0

    def update_debug_mode(self):
        self.debug_flag = self.debug_mode_switch.get() == 0
        print(self.debug_flag)

    def update_side(self):
        if self.side_switch.get() == 0:
            self.side = 'left'
        else:
            self.side = 'right'

    def calc_hub_alignment(self):
        def content(frame):
            ctk.CTkLabel(frame, text='Calculating hub alignment...', font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
        self.setup_screen('Processing Data', content, home_button=False)
        self.master.update()

        def compute_alignment():
            try:
                self.get_hub_calibration()
                scan_resultsL = scan_resultsR = MH.main(self.calibrationL, self.hub_scan_fileA, self.auto_flag, self.scan_type, ui=self)
                # scan_resultsR = MH.main(self.calibrationR, self.hub_scan_fileA, self.auto_flag, self.scan_type, ui=self)
                if isinstance(scan_resultsR, dict) and isinstance(scan_resultsL, dict):
                    self.toe_angleL = scan_resultsL.get("toe_angle", "N/A")
                    self.camber_angleL = scan_resultsL.get("camber_angle", "N/A")
                    self.toe_angleR = scan_resultsR.get("toe_angle", "N/A")
                    self.camber_angleR = scan_resultsR.get("camber_angle", "N/A")
                    self.total_toe = self.toe_angleR - self.toe_angleL if isinstance(self.toe_angleL, (int, float)) and isinstance(self.toe_angleR, (int, float)) else "N/A"
                    self.master.after(0, self.show_hub_results)
                else:
                    self.master.after(0, lambda: messagebox.showerror("Error", "Invalid scan results"))
            except Exception as e:
                self.master.after(0, lambda e=e: messagebox.showerror("Error", f"Scan failed: {e}"))

        # Keep UI responsive by scheduling periodic updates
        def update_ui():
            self.master.update()
            if threading.active_count() > 1:  # Check if background thread is still running
                self.master.after(10, update_ui)  # Schedule next update in 10ms

        # Start computation in a background thread
        threading.Thread(target=compute_alignment, daemon=True).start()
        self.master.after(100, update_ui)

    def show_hub_results(self):
        def content(frame):
            try:
                self.save_hub_results()
                self.print_hub_results()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save or print results: {e}")
            ctk.CTkLabel(frame, text="Measured Hub Alignment", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 10))
            ctk.CTkLabel(frame, text=f'Axle ID: {self.axle_id}', font=ctk.CTkFont(size=20, weight="bold")).pack(pady=(20, 10))
            results = (f'Left Toe:\t\t{self.toe_angleL}°\nLeft Camber:\t{self.camber_angleL}°\n'
                       f'Right Toe:\t{self.toe_angleR}°\nRight Camber:\t{self.camber_angleR}°\n'
                       f'Total Toe:\t{self.total_toe}°')
            ctk.CTkLabel(frame, text=results, font=ctk.CTkFont(size=18), justify="left", anchor="w").pack(pady=(20, 10))
            ctk.CTkButton(frame, text="Measure another axle", command=self.measure_hub).pack(pady=(10, 20))
            ctk.CTkButton(frame, text='Redo calculation in Manual Mode', command=lambda: [setattr(self, 'auto_flag', False), self.calc_hub_alignment()]).pack(pady=(10, 20))
            self.master.bind("<Return>", lambda event: self.measure_hub())
        self.setup_screen("Results", content)

    def save_hub_results(self):
        df = pd.read_csv(self.axle_database_path, dtype=str)
        df.loc[df["Axle ID"] == self.axle_id, ["Left Toe", "Left Camber", "Right Toe", "Right Camber", "Total Toe", "Date Scanned"]] = [
            self.toe_angleL, self.camber_angleL, self.toe_angleR, self.camber_angleR, self.total_toe, date.today()
        ]
        df.to_csv(self.axle_database_path, index=False)
        self.update_status(f"Scan results saved for Axle ID {self.axle_id}")

    def print_hub_results(self):
        pdf_path = os.path.join(r"C:\Users\Public\CapstoneUI", f"{self.axle_id}.pdf")
        os.makedirs(os.path.dirname(pdf_path), exist_ok=True)
        c = canvas.Canvas(pdf_path, pagesize=(2 * 72, 1 * 72))
        c.setFont("Courier", 8)
        text = c.beginText(0.25 * 72, 0.85 * 72)
        text.setLeading(10)
        for line in [f"Axle ID: {self.axle_id}", f"Left Toe: {self.toe_angleL}°", f"Left Camber: {self.camber_angleL}°", 
                     f"Right Toe: {self.toe_angleR}°", f"Right Camber: {self.camber_angleR}°", f"Total Toe: {self.total_toe}°"]:
            text.textLine(line)
        c.drawText(text)
        c.save()
        webbrowser.open(pdf_path)


    # Measure arm
    def show_arm_type_selection(self):
        self.load_arm_types()
        def content(frame):
            ctk.CTkLabel(frame, text="Select Arm Type", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
            values = sorted(list(self.arm_types.keys())) + ["Create New..."]
            self.arm_type_combo = ctk.CTkComboBox(frame, values=values, width=self.default_button_size)
            self.arm_type_combo.pack(pady=(0, 20))
            ctk.CTkButton(frame, text="Next", command=self.handle_arm_type_selection, width=self.default_button_size).pack(pady=(0, 20))

        self.setup_screen("TorFlex Axle — Select Arm Type", content)

    def handle_arm_type_selection(self):
        selected = self.arm_type_combo.get()
        if selected == "Create New...":
            self.open_create_arm_type()
        else:
            parts = selected.split('_')
            if len(parts) < 3:
                messagebox.showerror("Error", "Invalid arm type name format.")
                return
            arm_name = '_'.join(parts[:-2])  # Allow for names with underscores
            self.side = parts[-2].lower()
            self.scan_type = parts[-1].lower()
            if self.side not in ['left', 'right'] or self.scan_type not in ['real', 'sim']:
                messagebox.showerror("Error", "Invalid side or scan type in arm type name.")
                return
            self.selected_arm_type = selected
            self.show_arm_scan_screen()

    def open_create_arm_type(self):
        def content(frame):
            # Create a scrollable frame to hold all content, ensuring accessibility
            # regardless of window size. This prevents overflow and hidden elements
            # by allowing vertical scrolling when needed. We set it to fill and expand
            # to utilize the available space efficiently, improving usability on
            # smaller screens or with many bounding box entries.
            scrollable_frame = ctk.CTkScrollableFrame(frame)
            scrollable_frame.pack(fill=ctk.BOTH, expand=True, pady=(0, 20))
            
            ctk.CTkLabel(scrollable_frame, text="Create New Arm Type", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
            
            self.new_arm_name_entry = ctk.CTkEntry(scrollable_frame, placeholder_text="Arm Type Name (e.g., Standard)", width=self.default_button_size)
            self.new_arm_name_entry.pack(pady=(0, 10))
            
            self.new_side_combo = ctk.CTkComboBox(scrollable_frame, values=["right", "left"], width=self.default_button_size)
            self.new_side_combo.pack(pady=(0, 10))
            
            self.new_scan_type_combo = ctk.CTkComboBox(scrollable_frame, values=["real", "sim"], width=self.default_button_size)
            self.new_scan_type_combo.pack(pady=(0, 10))
            
            self.bbox_entries = {}
            bbox_keys = ["raw_trim", "inner_bar", "spindle_coarse", "spindle_fine"]
            coords = ['x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max']
            default_vals = ['-NO_LIMIT', 'NO_LIMIT', '-NO_LIMIT', 'NO_LIMIT', '-NO_LIMIT', 'NO_LIMIT']
            
            for key in bbox_keys:
                bbox_frame = ctk.CTkFrame(scrollable_frame)
                bbox_frame.pack(pady=(20, 0), fill=ctk.X)
                ctk.CTkLabel(bbox_frame, text=f"{key} Bounding Box:", font=ctk.CTkFont(weight="bold")).pack(anchor="w")
                coord_frame = ctk.CTkFrame(bbox_frame)
                coord_frame.pack(fill=ctk.X)
                self.bbox_entries[key] = {}
                for i, coord in enumerate(coords):
                    sub_frame = ctk.CTkFrame(coord_frame)
                    sub_frame.pack(side=ctk.LEFT, padx=5, expand=True, fill=ctk.X)
                    ctk.CTkLabel(sub_frame, text=coord).pack(anchor="w")
                    entry = ctk.CTkEntry(sub_frame, width=100)
                    entry.insert(0, default_vals[i])
                    entry.pack()
                    self.bbox_entries[key][coord] = entry

            self.rotation_entries = {}
            rotation_keys = ['Set 1', 'Set 2', 'Set 3']
            axes = ['X', 'Y', 'Z']
            default_vals = [0, 0, 0]

            for key in rotation_keys:
                rotation_frame = ctk.CTkFrame(scrollable_frame)
                rotation_frame.pack(pady=(20, 0), fill=ctk.X)
                ctk.CTkLabel(rotation_frame, text=f"{key} Rotations", font = ctk.CTkFont(weight='bold')).pack(anchor='w')
                axes_frame = ctk.CTkFrame(rotation_frame)
                axes_frame.pack(fill=ctk.X)
                self.rotation_entries[key] = {}
                for i, axis in enumerate(axes):
                    sub_frame = ctk.CTkFrame(axes_frame)
                    sub_frame.pack(side=ctk.LEFT, padx=5, expand=True, fill=ctk.X)
                    ctk.CTkLabel(sub_frame, text=axis).pack(anchor='w')
                    entry = ctk.CTkEntry(sub_frame, width=100)
                    entry.insert(0, default_vals[i])
                    entry.pack()
                    self.rotation_entries[key][axis] = entry
            
            ctk.CTkButton(scrollable_frame, text="Save", command=self.save_new_arm_type, width=self.default_button_size).pack(pady=(20, 0))

        self.setup_screen("TorFlex Axle — Create New Arm Type", content)

    def save_new_arm_type(self):
        name = self.new_arm_name_entry.get().strip()
        side = self.new_side_combo.get()
        scan_type = self.new_scan_type_combo.get()
        if not name or not side or not scan_type:
            messagebox.showerror("Error", "Please fill all fields.")
            return
        
        key = f"{name}_{side}_{scan_type}"
        if key in self.arm_types:
            messagebox.showerror("Error", "Arm type already exists.")
            return
        
        bboxes = {}
        coords = ['x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max']
        try:
            for bbox_key in self.bbox_entries:
                vals = []
                for coord in coords:
                    v = self.bbox_entries[bbox_key][coord].get().strip()
                    if v == 'NO_LIMIT':
                        vals.append(2000.0)
                    elif v == '-NO_LIMIT':
                        vals.append(-2000.0)
                    else:
                        vals.append(float(v))
                bboxes[bbox_key] = vals
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid bbox value: {e}. Use numbers or ±NO_LIMIT.")
            return
        
        rotations = {}
        axes = ['X', 'Y', 'Z']
        try:
            for set_key in self.rotation_entries:
                vals = []
                for axis in axes:
                    v = self.rotation_entries[set_key][axis].get().strip()
                    vals.append(float(v))
                rotations[set_key] = vals
        except ValueError as e:
            messagebox.showerror("Error", f'Invalid rotation value: {e}.')
        
        full_dict = {'bboxes': bboxes, 'rotations': rotations}

        self.arm_types[key] = full_dict
        self.save_arm_types()
        self.update_status(f"New arm type '{key}' saved.")
        self.selected_arm_type = key
        self.side = side
        self.scan_type = scan_type
        self.show_arm_scan_screen()

    def show_arm_scan_screen(self):
        def content(frame):
            ctk.CTkLabel(frame, text=f"Arm ID: {self.arm_id}", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
            
            test_frame = ctk.CTkFrame(frame)  # Sub-frame to hold label and combo side-by-side for alignment
            test_frame.pack(pady=(40, 0))  # Pack where the combo was, maintaining vertical spacing
            ctk.CTkLabel(test_frame, text="Test #:", font=ctk.CTkFont(size=18)).pack(side=ctk.LEFT, padx=(0, 10))  # Label to the left, with padding for spacing
            test_nums = [f"{i:02d}" for i in range(1, 100)]  # Generates ['01', '02', ..., '99']
            test_nums.insert(0, 'None: 00')
            self.test_num_combo = ctk.CTkComboBox(test_frame, values=test_nums, width=200)
            self.test_num_combo.pack(pady=(40, 0))
            self.test_num_combo.set('None: 00')  # Optional: Set default to '01' for convenience
            
            if self.scan_type == 'real':
                ctk.CTkButton(frame, text="Start Scanner", command=self.run_scanner, width=200).pack(pady=(40, 0))
            else:
                ctk.CTkLabel(frame, text="Simulation mode: Use existing scan file.", font=ctk.CTkFont(size=18)).pack(pady=(20, 0))
            scan_frame = ctk.CTkFrame(frame)
            scan_frame.pack(pady=(40, 0))
            ctk.CTkButton(scan_frame, text="Measure from existing scan:", command=self.validate_file_and_start, width=200).pack(side=ctk.LEFT, padx=(0, 10))
            self.existing_scan_entry = ctk.CTkEntry(scan_frame, placeholder_text="enter scan file path", width=300)
            self.existing_scan_entry.pack(side=ctk.LEFT)
            mode_frame = ctk.CTkFrame(frame)
            mode_frame.pack(pady=(20, 0))
            ctk.CTkLabel(mode_frame, text="Debug Mode:", font=ctk.CTkFont(size=18)).pack(side=ctk.LEFT, padx=(0, 10))
            self.debug_mode_switch = ctk.CTkSwitch(mode_frame, text="Debug/Auto", command=self.update_debug_mode)
            self.debug_mode_switch.pack(side=ctk.LEFT)
            self.debug_flag = self.debug_mode_switch.get() == 0
            # Removed: side_switch
            # Removed: debug_switch (assuming kept if present, but not in truncated)
            ctk.CTkButton(frame, text="Back", command=self.measure_arm, width=200).pack(pady=(40, 0))
            self.master.bind("<Return>", lambda event: self.run_scanner() if self.scan_type == 'real' else None)
        self.setup_screen("TorFlex Axle — Measure Arm Alignment", content)

    def calc_arm_alignment(self):
        if self.selected_arm_type is None:
            messagebox.showerror("Error", "No arm type selected.")
            return
        arm_type_data = self.arm_types.get(self.selected_arm_type)
        if arm_type_data is None:
            messagebox.showerror("Error", "Selected arm type not found in database.")
            return
        bboxes = arm_type_data.get('bboxes', {})
        rotations = arm_type_data.get('rotations', [])
        if isinstance(rotations, dict):  # Old format with per-side rotations
            rotations = rotations.get(self.side, [])
        def content(frame):
            ctk.CTkLabel(frame, text='Calculating crank arm alignment...', font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
        self.setup_screen('Processing Data', content, home_button=False)
        self.master.update()

        def compute_alignment():
            try:
                scan_results = MA.main(self.arm_scan_fileA, self.scan_type, side=self.side, ui=self, debug_flag=self.debug_flag, bboxes_dict=bboxes, rotations_list=rotations)
                if isinstance(scan_results, dict):
                    self.bar_toe = scan_results.get("bar_toe", "N/A")
                    self.bar_camber = scan_results.get("bar_camber", "N/A")
                    self.spindle_toe = scan_results.get("spindle_toe", "N/A")
                    self.spindle_camber = scan_results.get("spindle_camber", "N/A")
                    self.toe = scan_results.get("toe", "N/A")
                    self.camber = scan_results.get("camber", "N/A")
                    self.total_angle = scan_results.get("total_misalign", "N/A")
                    self.master.after(0, self.show_arm_results)
                else:
                    self.master.after(0, lambda: messagebox.showerror("Error", "Invalid scan results"))
            except Exception as e:
                self.master.after(0, lambda e=e: messagebox.showerror("Error", f"Scan failed: {e}"))

        # Local function to keep UI responsive during computation
        # This periodically updates the Tkinter main loop to process events,
        # preventing the UI from freezing while the background thread runs.
        # It checks if threads are still active to avoid unnecessary scheduling,
        # promoting efficiency in a production environment where UI responsiveness
        # impacts user experience on the production floor.
        def update_ui():
            self.master.update()
            if threading.active_count() > 1:  # Check if background thread is still running
                self.master.after(10, update_ui)  # Schedule next update in 10ms for smooth responsiveness

        # Start computation in a background thread to avoid blocking the UI,
        # ensuring the application remains interactive during potentially long-running
        # alignment calculations, which is critical for efficiency in quality control processes.
        threading.Thread(target=compute_alignment, daemon=True).start()
        self.master.after(100, update_ui)

    def calc_repeated_arm_alignment(self, scan_text):
        if self.selected_arm_type is None:
            messagebox.showerror("Error", "No arm type selected.")
            return
        arm_type_data = self.arm_types.get(self.selected_arm_type)
        if arm_type_data is None:
            messagebox.showerror("Error", "Selected arm type not found in database.")
            return
        bboxes = arm_type_data.get('bboxes', {})
        rotations = arm_type_data.get('rotations', [])
        if isinstance(rotations, dict):  # Old format with per-side rotations
            rotations = rotations.get(self.side, [])
        def content(frame):
            ctk.CTkLabel(frame, text=f'Calculating crank arm alignment for arm {scan_text}...', font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
        self.setup_screen('Processing Data', content, home_button=False)
        self.master.update()

        def compute_alignment():
            try:
                scan_results = MA.main(self.arm_scan_fileA, self.scan_type, side=self.side, ui=self, bboxes_dict=bboxes, rotations_list=rotations)
                if isinstance(scan_results, dict):
                    self.bar_toe = scan_results.get("bar_toe", "N/A")
                    self.bar_camber = scan_results.get("bar_camber", "N/A")
                    self.spindle_toe = scan_results.get("spindle_toe", "N/A")
                    self.spindle_camber = scan_results.get("spindle_camber", "N/A")
                    self.toe = scan_results.get("toe", "N/A")
                    self.camber = scan_results.get("camber", "N/A")
                    self.total_angle = scan_results.get("total_misalign", "N/A")

                    self.toe_arr = np.append(self.toe_arr, self.toe)
                    self.camber_arr = np.append(self.camber_arr, self.camber)
                    self.total_angle_arr = np.append(self.total_angle_arr, self.total_angle)

                    if (self.index + 1 >= self.scan_count):
                        self.master.after(0, self.save_repeated_arm_results, scan_text)
                        self.master.after(0, self.show_repeated_arm_results, scan_text)
                    else:
                        self.master.after(0, self.save_repeated_arm_results, scan_text)
                        return
                else:
                    self.master.after(0, lambda: messagebox.showerror("Error", "Invalid scan results"))
                    self.total_scans -= 1
                    return
            except Exception as e:
                self.master.after(0, lambda e=e: messagebox.showerror("Error", f"Scan failed: {e}"))

        compute_alignment()
    
    def show_arm_results(self):
        def content(frame):
            try:
                ctk.CTkLabel(frame, text="Measured Arm Alignment", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 10))
                ctk.CTkLabel(frame, text=f'Arm ID: {self.arm_id}', font=ctk.CTkFont(size=20, weight="bold")).pack(pady=(20, 10))
                results = (f'Total Toe:\t{self.toe:.3f}°\nTotal Camber:\t{self.camber:.3f}°\nTotal Angle:\t{self.total_angle:.3f}°')
                ctk.CTkLabel(frame, text=results, font=ctk.CTkFont(size=18), justify="left", anchor="w").pack(pady=(20, 10))
                ctk.CTkButton(frame, text="Measure another arm", command=self.measure_arm).pack(pady=(10, 20))
                ctk.CTkButton(frame, text='Redo calculation in Manual Mode', command=lambda: [setattr(self, 'debug_flag', False), self.calc_arm_alignment()]).pack(pady=(10, 20))
                self.master.bind("<Return>", lambda event: self.measure_arm())
                self.save_arm_results()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save or print results: {e}")
        
        self.setup_screen("Results", content)

    def show_repeated_arm_results(self, scan_text):
        def content(frame):
            ctk.CTkLabel(frame, text="Measured Arm Alignment", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 10))
            ctk.CTkLabel(frame, text=f'Arm ID: {self.arm_id}', font=ctk.CTkFont(size=20, weight="bold")).pack(pady=(20, 10))
            results = (f'Average Toe:\t{self.toe_avg:.4f}° ± {self.toe_std:.4f}\nAverage Camber:\t{self.camber_avg:.4f} ± {self.camber_std:.4f}°\nAverage Total Angle:\t{self.total_angle_avg:.4f}° ± {self.total_angle_std:.4f}')
            ctk.CTkLabel(frame, text=results, font=ctk.CTkFont(size=18), justify="center", anchor="w").pack(pady=(20, 10))
            ctk.CTkButton(frame, text="Measure another arm", command=self.measure_arm).pack(pady=(10, 20))
            ctk.CTkButton(frame, text='Redo calculation in Manual Mode', command=lambda: [setattr(self, 'debug_flag', False), self.calc_arm_alignment()]).pack(pady=(10, 20))
            self.master.bind("<Return>", lambda event: self.measure_arm())

        self.toe_avg = np.mean(self.toe_arr)
        self.camber_avg = np.mean(self.camber_arr)
        self.total_angle_avg = np.mean(self.total_angle_arr)

        self.toe_std = np.std(self.toe_arr)
        self.camber_std = np.std(self.camber_arr)
        self.total_angle_std = np.std(self.camber_arr)

        self.master.after(0, self.save_repeated_arm_avg_results)
        self.setup_screen("Results", content)

    def save_arm_results(self):
        os.makedirs(os.path.dirname(self.arm_database_path), exist_ok=True)
        self.initialize_csv(self.arm_database_path, ["Arm ID", "Test #", "Arm Type", "Toe (deg)", "Camber (deg)",
                                                    "Bar Toe (deg)", "Bar Camber (deg)", "Spindle Toe (deg)", "Spindle Camber (deg)",
                                                    "Total Relative Angle (deg)", "Date Scanned (yyyy-mm-dd_hhmmss)"])
        print('Arm Type Selected: ', self.selected_arm_type)
        
        rows = []
        with open(self.arm_database_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # print(row)
                rows.append(row)
        
        exists = False
        for row in rows:
            if row.get('Arm ID') == self.arm_id and row.get('Test #') == self.test_num:
                exists = True
                break  # Found match; will update this row below
        
        # Prepare the new/updated row with all fields, including Test Num
        new_row = {
            "Arm ID": self.arm_id,
            "Test #": self.test_num,
            "Arm Type": self.selected_arm_type,
            "Toe (deg)": self.toe,
            "Camber (deg)": self.camber,
            "Bar Toe (deg)": self.bar_toe,
            "Bar Camber (deg)": self.bar_camber,
            "Spindle Toe (deg)": self.spindle_toe,
            "Spindle Camber (deg)": self.spindle_camber,
            "Total Relative Angle (deg)": self.total_angle,
            "Date Scanned (yyyy-mm-dd_hhmmss)": datetime.now().strftime("%Y-%m-%d_%H%M%S")
        }
        
        if exists:
            # Update the matching row in-place for efficiency (avoids full rewrite if possible, but we rewrite anyway)
            print(f'Overwriting existing row for Arm ID {self.arm_id} and Test # {self.test_num}')
            for row in rows:
                if row['Arm ID'] == self.arm_id and row['Test #'] == self.test_num:
                    row.update(new_row)
                    break
        else:
            # Append new row for this test
            print(f'Appending new row for Arm ID {self.arm_id} and Test # {self.test_num}')
            rows.append(new_row)
        
        # Write back all rows to CSV (atomic rewrite ensures data integrity)
        with open(self.arm_database_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=["Arm ID", "Test #", "Arm Type", "Toe (deg)", "Camber (deg)",
                                                    "Bar Toe (deg)", "Bar Camber (deg)", "Spindle Toe (deg)", "Spindle Camber (deg)",
                                                    "Total Relative Angle (deg)", "Date Scanned (yyyy-mm-dd_hhmmss)"])
            writer.writeheader()
            writer.writerows(rows)
        
        self.update_status(f"Scan results saved for Arm ID {self.arm_id} (Test #: {self.test_num})")

    def save_repeated_arm_results(self, scan_text):
        df = pd.read_csv(self.arm_database_path, dtype=str)
        df.loc[df["Arm ID"] == scan_text, ["Bar Toe", "Bar Camber",
                                            "Spindle Toe", "Spindle Camber",
                                            "Toe", "Camber",
                                            "Total Relative Angle", "Date Scanned"]] = [self.bar_toe, self.bar_camber,
                                                                                            self.spindle_toe, self.spindle_camber,
                                                                                            self.toe, self.camber,
                                                                                            self.total_angle, date.today()]
        df.to_csv(self.arm_database_path, index=False)
        self.update_status(f"Scan results saved for Arm ID {scan_text}")

    def save_repeated_arm_avg_results(self):
        df = pd.read_csv(self.arm_database_path, dtype=str)
        df.loc[df["Arm ID"] == self.arm_id, ["Toe", "Camber",
                                                "Total Relative Angle", "Date Scanned"]] = [self.toe_avg, self.camber_avg,
                                                                                            self.total_angle_avg, date.today()]
        df.to_csv(self.arm_database_path, index=False)
        self.update_status(f"Scan results saved for Arm ID {self.arm_id}")


    # Calibrate hub
    def input_calibration_axle_data(self):
        def content(frame):
            ctk.CTkLabel(frame, text='Enter known values of calibration axle (degrees):', font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(20, 40))
            self.toeL_entry = ctk.CTkEntry(frame, placeholder_text="Left Toe", width=300)
            self.toeL_entry.pack(pady=(0, 20))
            self.camberL_entry = ctk.CTkEntry(frame, placeholder_text="Left Camber", width=300)
            self.camberL_entry.pack(pady=(0, 20))
            self.toeR_entry = ctk.CTkEntry(frame, placeholder_text="Right Toe", width=300)
            self.toeR_entry.pack(pady=(0, 20))
            self.camberR_entry = ctk.CTkEntry(frame, placeholder_text="Right Camber", width=300)
            self.camberR_entry.pack(pady=(0, 20))
            ctk.CTkButton(frame, text="Start Calibration Scan", command=self.start_calibration_scan, width=200).pack(pady=(40, 0))
            self.master.bind("<Return>", lambda event: self.start_calibration_scan())
            mode_frame = ctk.CTkFrame(frame)
            mode_frame.pack(pady=(20, 0))
            ctk.CTkLabel(mode_frame, text="Manual Mode:", font=ctk.CTkFont(size=18)).pack(side=ctk.LEFT, padx=(0, 10))
            self.auto_mode_switch = ctk.CTkSwitch(mode_frame, text="Auto/Manual", command=self.update_auto_mode)
            self.auto_mode_switch.pack(side=ctk.LEFT)
            self.auto_flag = self.auto_mode_switch.get() == 0
        self.setup_screen("Hub Alignment Calibration", content)

    def start_calibration_scan(self):
        try:
            self.offset_toeL, self.offset_camberL = float(self.toeL_entry.get()), float(self.camberL_entry.get())
            self.offset_toeR, self.offset_camberR = float(self.toeR_entry.get()), float(self.camberR_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values for toe and camber.")
            return

        def content(frame):
            ctk.CTkLabel(frame, text="Scanning calibration axle...", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
        self.setup_screen("Hub Alignment Calibration", content, home_button=False)
        self.master.update()

        data = PS.perform_scan().astype(float)
        for i in data:
            i = (i - 2**15) * .0102
        np.savetxt(self.temp_scan_pathA, data, delimiter=',', header='X Y Z')
        self.hub_scan_fileA = self.temp_scan_pathA
        self.scan_type = 'real'
        self.calc_calibration()

    def calc_calibration(self):
        def content(frame):
            ctk.CTkLabel(frame, text="Processing calibration...", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 40))
        self.setup_screen("Hub Alignment Calibration", content, home_button=False)

        def compute_calibration():
            try:
                scan_resultsL = scan_resultsR = MH.main(self.calibrationL, self.hub_scan_fileA, self.auto_flag, self.scan_type, ui=self)
                # scan_resultsR = MH.main(self.calibrationR, self.hub_scan_fileA, self.auto_flag, self.scan_type, ui=self)
                if isinstance(scan_resultsR, dict) and isinstance(scan_resultsL, dict):
                    self.rotationL_aboutx = scan_resultsL.get("camber_angle", 0) - self.offset_camberL
                    self.rotationL_abouty = scan_resultsL.get("toe_angle", 0) - self.offset_toeL
                    self.rotationR_aboutx = scan_resultsR.get("camber_angle", 0) - self.offset_camberR
                    self.rotationR_abouty = scan_resultsR.get("toe_angle", 0) - self.offset_toeR
                    self.master.after(0, self.show_calibration_results)
                else:
                    self.master.after(0, lambda: messagebox.showerror("Error", "Invalid calibration results"))
            except Exception as e:
                self.master.after(0, lambda e=e: messagebox.showerror("Error", f"Calibration failed: {e}"))

        # Keep UI responsive by scheduling periodic updates
        def update_ui():
            self.master.update()
            if threading.active_count() > 1:  # Check if background thread is still running
                self.master.after(10, update_ui)  # Schedule next update in 10ms

        # Start computation in a background thread
        threading.Thread(target=compute_calibration, daemon=True).start()
        self.master.after(100, update_ui)

    def show_calibration_results(self):
        def content(frame):
            try:
                self.save_calibration()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save calibration: {e}")
            ctk.CTkLabel(frame, text="Results — offsets will be applied to all future measurements:", font=ctk.CTkFont(size=24, weight="bold")).pack(pady=(20, 10))
            results = f"Left X offset: {self.rotationL_aboutx}°\nLeft Y offset: {self.rotationL_abouty}°\nRight X offset: {self.rotationR_aboutx}°\nRight Y offset: {self.rotationR_abouty}°"
            ctk.CTkLabel(frame, text=results, font=ctk.CTkFont(size=18)).pack(pady=(10, 20))
        self.setup_screen("Hub Alignment Calibration", content)

    def save_calibration(self):
        self.initialize_csv(self.calibration_path, ["Left Rotation About X", "Left Rotation About Y", "Right Rotation About X", "Right Rotation About Y", "Date"])
        pd.concat([pd.read_csv(self.calibration_path, dtype=str), pd.DataFrame([{
            "Left Rotation About X": self.rotationL_aboutx, "Left Rotation About Y": self.rotationL_abouty,
            "Right Rotation About X": self.rotationR_aboutx, "Right Rotation About Y": self.rotationR_abouty,
            "Date": date.today()
        }])], ignore_index=True).to_csv(self.calibration_path, index=False)
        self.update_status("Calibration saved")

    def reset_calibration(self):
        try:
            os.remove(self.calibration_path)
            self.get_hub_calibration()
            self.calibrate_hub()
        except FileNotFoundError:
            self.update_status(f"Calibration file {self.calibration_path} not found, resetting to default")
            self.get_hub_calibration()
            self.calibrate_hub()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to reset calibration: {e}")

    def initialize_csv(self, path, columns):
        if not os.path.exists(path):
            pd.DataFrame(columns=columns).to_csv(path, index=False)

if __name__ == "__main__":
    root = ctk.CTk()
    q = queue.Queue(maxsize=1)
    app = Dexter_Capstone_UI(root, q)
    root.mainloop()