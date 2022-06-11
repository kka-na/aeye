import tkinter as tk
import tkinter.ttk as ttk
import cankey

class Compare:
    def __init__(self):
        self.window = 
        self.result = ''

        self._past = cankey.DATA.copy()
        self._current = cankey.DATA.copy()

    def init_UI(self):
        self.window = tk.Tk()
        self.window.rowconfigure([0], weight=1, minsize=100)
        self.window.columnconfigure([0,1], weight=1, minsize=100)

        button_frame = ttk.Frame(master=self.window, width=100, height=100)
        output_frame = ttk.Frame(master=self.window, width=500, height=100)

        button_frame.grid(row=0, column=0)
        output_frame.grid(row=0, column=1)

        button_store = ttk.Button(master=button_frame, text="Store Signals", command=self.store)
        button_compare = ttk.Button(master=button_frame,text="Compare Signals", command=self.compare)

        output = tk.Text(output_frame)
        scollb = ttk.Scrollbar(command=output.yview)
        scollb.grid(row=0, column=2, sticky='nsew')
        output['yscrollcommand'] = scollb.set

        output.insert(tk.END, self.result)

        output.pack()
        button_store.pack()
        button_compare.pack(padx=5, pady=10)

    def store(self):
        

    def compare(self):


        window.mainloop()


