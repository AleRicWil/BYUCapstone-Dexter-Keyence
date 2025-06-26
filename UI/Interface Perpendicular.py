import customtkinter as ctk
from tkinter import messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import pandas as pd
import processing
import tempfile
import os
import tkinter as tk
from tkinter import filedialog
from reportlab.pdfgen import canvas
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfbase import pdfmetrics
import webbrowser
import csv
import threading
from datetime import date

# Go to get_axle_spec_data() and print_results() and check the filepaths for where to get the database and where to save label pdf
# Ensure that the database is not open on the computer

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class DexterAxolotlsApp:
    def __init__(self, master):
        self.master = master
        master.title("Dexter Axolotls 2025")
        master.geometry("800x600")
        self.open_setup_scan()
    
    def open_setup_scan(self):
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Axle ID", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))

        self.barcode_entry = ctk.CTkEntry(frame, placeholder_text="Enter Barcode", width=300)
        self.barcode_entry.pack(pady=(0, 20))
        self.master.bind("<Return>", lambda event: self.save_axleid())

        enter_button = ctk.CTkButton(frame, text="Enter", command=self.save_axleid, width=200)
        enter_button.pack(pady=(0, 20))

        calibration_button = ctk.CTkButton(frame, text="Calibrate", command=self.get_reference_axle_data, width=200)
        calibration_button.pack(pady=(0, 20))

    def save_axleid(self):
        self.master.unbind("<Return>")
        self.axle_id = self.barcode_entry.get().strip()

        if not self.axle_id:
            messagebox.showerror("Error", "Please enter an Axle ID")
            return

        # File path for the database CSV
        self.database_path = r"C:\Users\Public\CapstoneUI\Database.csv"

        # Check if the file exists; if not, create it with headers
        if not os.path.exists(self.database_path):
            df_headers = pd.DataFrame(columns=["Axle ID", "Left Toe", "Left Camber", "Right Toe", "Right Camber", "Total Toe", "Date Scanned"])
            df_headers.to_csv(self.database_path, index=False)  # Create the file with headers

        try:
            # Read the CSV file
            df_database = pd.read_csv(self.database_path, dtype=str)  # Read all columns as strings to avoid issues
            
            # Check if the Axle ID already exists
            if self.axle_id in df_database["Axle ID"].values:
                print(f"Axle ID {self.axle_id} already exists.")
            else:
                # Axle ID not found, add a new row
                new_entry = pd.DataFrame([{
                    "Axle ID": self.axle_id,
                    "Left Toe": "", "Left Camber": "",
                    "Right Toe": "", "Right Camber": "",
                    "Total Toe": "", "Date Scanned": ""
                }])

                # Append new row to the dataframe
                df_database = pd.concat([df_database, new_entry], ignore_index=True)
                # Save the updated CSV
                df_database.to_csv(self.database_path, index=False)
                print(f"Axle ID {self.axle_id} added to database.")

        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {e}")
            return
        
        self.get_calibration()

        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        id = ctk.CTkLabel(frame, text=f"Axle ID {self.axle_id}", font=ctk.CTkFont(size=24, weight="bold"))
        id.pack(pady=(20, 40))

        calibration_date = ctk.CTkLabel(frame, text=f"Last Calibrated: {self.calibration_date}", font=ctk.CTkFont(size=24, weight="bold"))
        calibration_date.pack(pady=(20, 40))

        start_scan = ctk.CTkButton(frame, text="Start Scan", command=self.start_scan, width=200)
        start_scan.pack(pady=(40, 0))
        self.master.bind("<Return>", lambda event: self.start_scan())

        back_button = ctk.CTkButton(frame, text="Back", command=self.open_setup_scan, width=200)
        back_button.pack(pady=(40, 0))

    def get_calibration(self):
        # File path for the database CSV
        self.calibration_path = r"C:\Users\Public\CapstoneUI\Calibration History.csv"

        # Check if the file exists; if not, create it with headers
        if not os.path.exists(self.calibration_path):
            df_headers = pd.DataFrame(columns=["Left Rotation About X", "Left Rotation About Y", "Right Rotation About X", "Right Rotation About Y", "Date"])
            df_headers.to_csv(self.calibration_path, index=False)  # Create the file with headers
            self.calibrationL = {"x": 0, "y": 0}
            self.calibrationR = {"x": 0, "y": 0}
            self.calibration_date = date.today()
            df_calibration = pd.read_csv(self.calibration_path, dtype=str)
            new_entry = pd.DataFrame({
                "Left Rotation About X": [0],
                "Left Rotation About Y": [0],
                "Right Rotation About X": [0],
                "Right Rotation About Y": [0],
                "Date": [date.today()],
                }) 
            df_calibration = pd.concat([df_calibration, new_entry], ignore_index=True)  # Append new row
            df_calibration.to_csv(self.calibration_path, index=False)  # Save updated CSV
        else:
            df_calibration = pd.read_csv(self.calibration_path)
            last_entry = df_calibration.iloc[-1].to_dict()
            self.calibration_date = last_entry['Date']
            self.calibrationL = {'x':last_entry['Left Rotation About X'],
                                 'y': last_entry['Left Rotation About Y']}
            self.calibrationR = {'x':last_entry['Right Rotation About X'],
                                 'y': last_entry['Right Rotation About Y']}

    def start_scan(self):
        self.master.unbind("<Return>")
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Scan in Process", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))

        self.master.update_idletasks()  # Force UI update

        # Run the scan in a separate thread to prevent blocking
        thread = threading.Thread(target=self.run_scan)
        thread.start()

    def run_scan(self):
        try:
            scan_resultsR = processing.main(self.calibrationR)
            scan_resultsL = processing.main(self.calibrationL)
            # scan_resultsR = {"toe_angle": 0.3, "camber_angle":0.2}
            # scan_resultsL = {"toe_angle": 0.4, "camber_angle":0.3}
            print("DEBUG: Scan results received:", scan_resultsR, scan_resultsL)

            if isinstance(scan_resultsR, dict):
                self.toe_angleL = scan_resultsL.get("toe_angle", "N/A")
                self.camber_angleL = scan_resultsL.get("camber_angle", "N/A")
                self.toe_angleR = scan_resultsR.get("toe_angle", "N/A")
                self.camber_angleR = scan_resultsR.get("camber_angle", "N/A")
                self.total_toe = self.find_total_toe(self.toe_angleL, self.toe_angleR)
                
                # Schedule UI update on the main thread
                self.master.after(0, self.show_scan_results)
            else:
                self.master.after(0, lambda: messagebox.showerror("Error", "Invalid scan results received."))

        except Exception as e:
            self.master.after(0, lambda e=e: messagebox.showerror("Error", f"Failed to show scan: {e}"))

    def find_total_toe(self, toe1, toe2):
        return toe1 + toe2

    def show_scan_results(self):
        self.master.unbind("<Return>")
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Scan Results", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 10))

        self.save_scan_results()
        self.print_results()

        results_text = f"Left Toe: {self.toe_angleL}°\nLeft Camber: {self.camber_angleL}°\nRight Toe: {self.toe_angleR}°\nRight Camber: {self.camber_angleR}°\nTotal Toe: {self.total_toe}°"
        
        results_label = ctk.CTkLabel(frame, text=results_text, font=ctk.CTkFont(size=18))
        results_label.pack(pady=(10, 20))

        restart_button = ctk.CTkButton(frame, text="Restart", command=self.open_setup_scan)
        restart_button.pack(pady=(10, 20))
        self.master.bind("<Return>", lambda event: self.open_setup_scan())

    def save_scan_results(self):
        csv_file = self.database_path
        temp_file = csv_file + ".tmp"  # Temporary file to avoid overwriting issues

        try:
            with open(csv_file, mode="r", newline="") as infile, open(temp_file, mode="w", newline="") as outfile:
                reader = csv.DictReader(infile)
                fieldnames = reader.fieldnames
                
                if "Right Toe" not in fieldnames or "Right Camber" not in fieldnames or "Left Toe" not in fieldnames or "Left Camber" not in fieldnames or "Total Toe" not in fieldnames or "Date Scanned" not in fieldnames:
                    raise ValueError("CSV file is missing column name")

                writer = csv.DictWriter(outfile, fieldnames=fieldnames)
                writer.writeheader()

                updated = False
                for row in reader:
                    if row["Axle ID"] == str(self.axle_id):  # Match Axle ID
                        row["Left Toe"] = self.toe_angleL
                        row["Left Camber"] = self.camber_angleL
                        row["Right Toe"] = self.toe_angleR
                        row["Right Camber"] = self.camber_angleR
                        row["Total Toe"] = self.total_toe
                        row["Date Scanned"] = date.today()
                        updated = True
                    writer.writerow(row)

            # Replace the old file with the updated one
            os.replace(temp_file, csv_file)

            if updated:
                print(f"Scan results saved for Axle ID {self.axle_id}")
            else:
                print(f"Axle ID {self.axle_id} not found in database")

        except Exception as e:
            print(f"Error writing CSV: {e}")

    def print_results(self):
        save_directory = r"C:\Users\Public\CapstoneUI" #Change to desired filepath
        
        pdf_filename = os.path.join(save_directory, f"{self.axle_id}.pdf")
        inch = 72 #points
        width, height = 2 * inch, 1 * inch
        
        c = canvas.Canvas(pdf_filename, pagesize=(width, height))

        c.setFont("Courier", 8)
        left_margin = 0.25 * inch
        top_margin = height - 0.15 * inch  # Start from the top
        line_height = 10  

        # Draw centered text
        c.drawString(left_margin, top_margin - 0 * line_height, f"Axle ID: {self.axle_id}")
        c.drawString(left_margin, top_margin - 1 * line_height, f"Left Toe: {self.toe_angleL}°")
        c.drawString(left_margin, top_margin - 2 * line_height, f"Left Camber: {self.camber_angleL}°")
        c.drawString(left_margin, top_margin - 3 * line_height, f"Right Toe: {self.toe_angleR}°")
        c.drawString(left_margin, top_margin - 4 * line_height, f"Right Camber: {self.camber_angleR}°")
        c.drawString(left_margin, top_margin - 5 * line_height, f"Total Toe: {self.total_toe}°")
        c.save()
        # Open the generated PDF
        webbrowser.open(pdf_filename)

    def get_reference_axle_data(self):
        self.master.unbind("<Return>")
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Enter Reference Axle Info", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))

        self.toeL_entry = ctk.CTkEntry(frame, placeholder_text="Enter Left Toe", width=300)
        self.toeL_entry.pack(pady=(0, 20))

        self.camberL_entry = ctk.CTkEntry(frame, placeholder_text="Enter Left Camber", width=300)
        self.camberL_entry.pack(pady=(0, 20))

        self.toeR_entry = ctk.CTkEntry(frame, placeholder_text="Enter Right Toe", width=300)
        self.toeR_entry.pack(pady=(0, 20))

        self.camberR_entry = ctk.CTkEntry(frame, placeholder_text="Enter Right Camber", width=300)
        self.camberR_entry.pack(pady=(0, 20))

        save_calibration = ctk.CTkButton(frame, text="Calibrate", command=self.start_calibration, width=200)
        save_calibration.pack(pady=(40, 0))
        self.master.bind("<Return>", lambda event: self.start_calibration())

        back_button = ctk.CTkButton(frame, text="Back", command=self.open_setup_scan, width=200)
        back_button.pack(pady=(40, 0))

    def start_calibration(self):
        self.master.unbind("<Return>")
        self.reference_toeL = float(self.toeL_entry.get().strip())
        self.reference_camberL = float(self.camberL_entry.get().strip())
        self.reference_toeR = float(self.toeR_entry.get().strip())
        self.reference_camberR = float(self.camberR_entry.get().strip())
        
        if not self.reference_toeL or not self.reference_camberL or not self.reference_toeR or not self.reference_camberR:
            messagebox.showerror("Error", "Please enter the reference axle's toe and camber.")
            return
        
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Calibration in Process", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))

        self.master.update_idletasks()  # Force UI update

        # Run the scan in a separate thread to prevent blocking
        thread = threading.Thread(target=self.run_calibration)
        thread.start()

    def run_calibration(self):
        self.master.unbind("<Return>")
        try:
            calibration = {"x":0, "y":0}
            scan_resultsR = processing.main(calibration)
            scan_resultsL = processing.main(calibration)
            # scan_resultsR = {"toe_angle": 0.5, "camber_angle":0.5}
            # scan_resultsL = {"toe_angle": 0.5, "camber_angle":0.5}
            print("DEBUG: Scan results received:", scan_resultsR, scan_resultsL)

            if isinstance(scan_resultsR, dict):
                toeL = scan_resultsL.get("toe_angle", "N/A")
                camberL = scan_resultsL.get("camber_angle", "N/A")
                toeR = scan_resultsR.get("toe_angle", "N/A")
                camberR = scan_resultsR.get("camber_angle", "N/A")
                
                self.rotationL_aboutx = camberL - self.reference_camberL
                self.rotationL_abouty = toeL - self.reference_toeL
                self.rotationR_aboutx = camberR - self.reference_camberR
                self.rotationR_abouty = toeR - self.reference_toeR

                # Schedule UI update on the main thread
                # print("DEBUG: Calling show_calibration_results()")
                self.master.after(0, self.show_calibration_results)
            else:
                self.master.after(0, lambda: messagebox.showerror("Error", "Invalid scan results received."))

        except Exception as e:
            self.master.after(0, lambda: messagebox.showerror("Error", "Failed to show scan results."))

    def show_calibration_results(self):
        self.master.unbind("<Return>")
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Calibration Results", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 10))

        self.save_calibration()

        results_text = f"Left Rotation About X: {self.rotationL_aboutx}°\nLeft Rotation About Y: {self.rotationL_abouty}°\nRight Rotation About X: {self.rotationR_aboutx}°\nRight Rotation About Y: {self.rotationR_abouty}°"
        
        results_label = ctk.CTkLabel(frame, text=results_text, font=ctk.CTkFont(size=18))
        results_label.pack(pady=(10, 20))

        restart_button = ctk.CTkButton(frame, text="Restart", command=self.open_setup_scan)
        restart_button.pack(pady=(10, 20))
        restart_button.bind("<Return>", lambda event: self.open_setup_scan())

    def save_calibration(self):
        # File path for the database CSV
        self.calibration_path = r"C:\Users\Public\CapstoneUI\Calibration History.csv"

        # Check if the file exists; if not, create it with headers
        if not os.path.exists(self.calibration_path):
            df_headers = pd.DataFrame(columns=["Left Rotation About X", "Left Rotation About Y", "Right Rotation About X", "Right Rotation About Y", "Date"])
            df_headers.to_csv(self.calibration_path, index=False)  # Create the file with headers

        try:
            # Read the CSV file
            df_calibration = pd.read_csv(self.calibration_path, dtype=str)  # Read all columns as strings to avoid issues
            
            # Axle ID not found, add a new row
            new_entry = pd.DataFrame({
                "Left Rotation About X": [self.rotationL_aboutx],
                "Left Rotation About Y": [self.rotationL_abouty],
                "Right Rotation About X": [self.rotationR_aboutx],
                "Right Rotation About Y": [self.rotationR_abouty],
                "Date": [date.today()],
                })
                
            df_calibration = pd.concat([df_calibration, new_entry], ignore_index=True)  # Append new row
            df_calibration.to_csv(self.calibration_path, index=False)  # Save updated CSV
            print("New Calibration Entry")

        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {e}")
            return

    def open_settings(self):
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Settings", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))

        back_button = ctk.CTkButton(frame, text="Back", command=self.create_main_page, width=200)
        back_button.pack(pady=(40, 0))

    def clear_window(self):
        for widget in self.master.winfo_children():
            widget.destroy()

if __name__ == "__main__":
    root = ctk.CTk()
    app = DexterAxolotlsApp(root)
    root.mainloop()