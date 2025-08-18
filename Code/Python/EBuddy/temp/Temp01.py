import tkinter as tk
from tkinter import ttk

# Create the main window
root = tk.Tk()
root.title("Months Tabs")

# Create a notebook (tab container)
notebook = ttk.Notebook(root)

# List of month names for the tabs
months = ["January", "February", "March", "April", "May"]

# Inspirational words for each month
inspirational_words = {
    "January": "❄️ Fresh Start, Snowfall, Resolutions, New Beginnings",
    "February": "💖 Love, Hearts, Warmth, Romance",
    "March": "🌱 Renewal, Buds, Awakening, Green Shoots",
    "April": "🌸 Blossom, Rain Showers, Joy, Growth",
    "May": "🌼 Bloom, Sunshine, Energy, Celebration"
}

# Create tabs for each month with inspirational words
for month in months:
    frame = ttk.Frame(notebook)
    label = ttk.Label(frame, text=inspirational_words[month], padding=10)
    label.pack(expand=True)
    notebook.add(frame, text=month)

# Pack the notebook into the main window
notebook.pack(expand=True, fill='both')

# Run the application
root.mainloop()
