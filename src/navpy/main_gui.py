#!/usr/bin/env python3
from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import tkinter as tk
from dataclasses import asdict, dataclass
from pathlib import Path
from tkinter import ttk, messagebox
from typing import Dict, Any

# ───────────────────────── config / persistence ──────────────────────────
CONFIG_PATH = Path.home() / ".param_launcher.json"
DEFAULT_ID = "0"


@dataclass
class Profile:
    ss: int = 1
    comm_type: str = "serial"  # serial | wifi | mavlink
    c: str = "tcp:127.0.0.1:5762"
    sc: str = "COM11"
    sb: str = "115200"
    wsp: str = "14551"
    wpp: str = "tcp://localhost:14500 tcp://localhost:14550 tcp://localhost:14552"

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Profile":
        return cls(**{**asdict(cls()), **d})

    def to_cli(self) -> list[str]:
        base = ["-c", self.c, "-ss", str(self.ss), "-nt", self.comm_type]
        if self.comm_type == "serial":
            base += ["-sc", self.sc, "-sb", self.sb]
        elif self.comm_type == "wifi":
            base += ["-wsp", self.wsp, "-wpp", *self.wpp.split()]
        return base


def _load_file() -> dict:
    if CONFIG_PATH.exists():
        try:
            return json.loads(CONFIG_PATH.read_text())
        except Exception as ex:
            logging.error("Cannot read %s: %s", CONFIG_PATH, ex)
    return {}


def load_profiles() -> tuple[dict[str, Profile], str]:
    raw = _load_file()
    last = raw.get("last", DEFAULT_ID)
    profs = {pid: Profile.from_dict(p) for pid, p in raw.items() if pid.isdigit()}
    profs.setdefault(DEFAULT_ID, Profile())
    return profs, last


def save_profile(pid: str, prof: Profile) -> None:
    store = _load_file()
    store["last"] = pid
    store[pid] = asdict(prof)
    CONFIG_PATH.write_text(json.dumps(store, indent=2))


# ───────────────────────────────── GUI ──────────────────────────────────
class Launcher(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("main.py launcher")
        self.resizable(False, False)

        # profiles
        self.profiles, self.current_id = load_profiles()
        self.profile: Profile = self.profiles[self.current_id]

        # Tk variables
        self.comm_type = tk.StringVar(value=self.profile.comm_type)
        self.ss_var = tk.StringVar(value=str(self.profile.ss))
        self.ss_var.trace_add("write", self._on_ss_change)

        # widgets
        self._build_ui()
        self._load_ui()

    # ───────────────────────── UI helpers ────────────────────────────
    @staticmethod
    def _add_labeled_entry(parent, row, text, width=25):
        ttk.Label(parent, text=text).grid(row=row, column=0, sticky="e")
        e = ttk.Entry(parent, width=width)
        e.grid(row=row, column=1, sticky="w")
        return e

    def _build_ui(self):
        row = 0
        idf = ttk.LabelFrame(self, text="Vehicle Sys-ID")
        idf.grid(row=row, column=0, columnspan=3, padx=4, pady=4, sticky="ew")
        ttk.Label(idf, text="-ss").grid(row=0, column=0, sticky="e")
        self.ss_spin = ttk.Spinbox(idf, from_=1, to=255, width=5,
                                   textvariable=self.ss_var)
        self.ss_spin.grid(row=0, column=1, sticky="w")

        row += 1
        conn = ttk.LabelFrame(self, text="Connection")
        conn.grid(row=row, column=0, columnspan=3, padx=4, pady=4, sticky="ew")
        self.c_entry = self._add_labeled_entry(conn, 0, "-c (TCP endpoint)")

        row += 1
        ttk.Label(self, text="Communication type:").grid(row=row, column=0, padx=4, sticky="w")
        ttk.Radiobutton(self, text="Serial", value="serial",
                        variable=self.comm_type, command=self._toggle).grid(row=row, column=1, sticky="w")
        ttk.Radiobutton(self, text="Wi-Fi", value="wifi",
                        variable=self.comm_type, command=self._toggle).grid(row=row, column=2, sticky="w")
        ttk.Radiobutton(self, text="MAVLink", value="mav",
                        variable=self.comm_type, command=self._toggle).grid(row=row, column=3, sticky="w")

        row += 1
        self.serial_frame = ttk.LabelFrame(self, text="Serial")
        self.serial_frame.grid(row=row, column=0, columnspan=3, padx=4, pady=4, sticky="ew")
        self.sc_entry = self._add_labeled_entry(self.serial_frame, 0, "-sc (COM port)", 10)
        self.sb_entry = self._add_labeled_entry(self.serial_frame, 1, "-sb (baud)", 10)

        row += 1
        self.wifi_frame = ttk.LabelFrame(self, text="Wi-Fi")
        self.wifi_frame.grid(row=row, column=0, columnspan=3, padx=4, pady=4, sticky="ew")
        self.wsp_entry = self._add_labeled_entry(self.wifi_frame, 0, "-wsp (server port)", 10)
        self.wpp_entry = self._add_labeled_entry(self.wifi_frame, 1, "-wpp (peers)", 45)

        row += 1
        btns = ttk.Frame(self)
        btns.grid(row=row, column=0, columnspan=3, pady=8)
        ttk.Button(btns, text="Run", command=self._run).grid(row=0, column=0, padx=4)
        ttk.Button(btns, text="Quit", command=self.destroy).grid(row=0, column=1, padx=4)

    # ───────────────────────── profile ⇆ widgets ───────────────────────
    def _widget_map(self):
        return {
            "c": self.c_entry,
            "sc": self.sc_entry,
            "sb": self.sb_entry,
            "wsp": self.wsp_entry,
            "wpp": self.wpp_entry,
        }

    def _load_ui(self):
        for key, widget in self._widget_map().items():
            widget.delete(0, tk.END)
            widget.insert(0, getattr(self.profile, key))
        self.comm_type.set(self.profile.comm_type)
        self.ss_var.set(str(self.profile.ss))  # ← keep Spinbox in sync
        self._toggle()

    def _collect_profile(self) -> Profile:
        kw = {k: w.get().strip() for k, w in self._widget_map().items()}
        kw.update(comm_type=self.comm_type.get(), ss=self.ss_var.get() or 0)
        return Profile(**kw)

    # ───────────────────────── callbacks ───────────────────────────────
    def _toggle(self):
        mode = self.comm_type.get()
        serial_mode = mode == "serial"
        wifi_mode = mode == "wifi"

        for child in self.serial_frame.winfo_children():
            child.state(["!disabled" if serial_mode else "disabled"])
        for child in self.wifi_frame.winfo_children():
            child.state(["!disabled" if wifi_mode else "disabled"])

    def _on_ss_change(self, *_):
        if not self.ss_var.get().isdigit():
            return
        new_id = self.ss_var.get()
        if new_id == self.current_id:
            return
        self.current_id = new_id
        self.profile = self.profiles.get(new_id, Profile(ss=int(new_id)))
        self._load_ui()

    # ───────────────────────── actions ────────────────────────────────
    def _run(self):
        self._save_current_profile()

        py = self._find_venv_python()
        cmd = [str(py), "-m", "navpy.main", *self.profile.to_cli()]

        try:
            flags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
            subprocess.Popen(cmd, creationflags=flags)  # returns immediately
        except Exception as exc:
            messagebox.showerror("Launch failed", str(exc))

    # ─── helper ────────────────────────────────────────────────────────
    def _save_current_profile(self) -> None:
        """Pull values from widgets → Profile → disk & RAM."""
        self.profile = self._collect_profile()
        save_profile(self.current_id, self.profile)
        self.profiles[self.current_id] = self.profile

    @staticmethod
    def _find_venv_python() -> Path:
        """
        Return the Python executable in './venv' **or** './.venv'.
        Falls back to the running interpreter if no venv is found.
        """
        root = Path(__file__).resolve().parents[2]  # repo root
        for venv_dir in ("venv", ".venv"):
            candidate = root / venv_dir / (
                "Scripts/python.exe" if os.name == "nt" else "bin/python")
            if candidate.exists():
                return candidate
        return Path(sys.executable)


# ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    Launcher().mainloop()
