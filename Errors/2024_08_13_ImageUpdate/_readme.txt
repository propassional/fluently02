Error in FLGUIMain:
try
	if os.path.exists(sm_in_loop.sm_graph_path) and os.path.getsize(sm_in_loop.sm_graph_path) > 0:
		# Before the above if, sometimes I got TclError here, try following code: with open('image.jpg', 'wb') as f: f.close()
		self.window_main[f'-{sm_in_loop.sm_gui.name}_IMAGE-'].update(sm_in_loop.sm_graph_path)


C:\Users\operator\.conda\envs\ml_env\Lib\site-packages\PySimpleGUI\PySimpleGUI.py
line 5884:
image = tk.PhotoImage(file=filename)