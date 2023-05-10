# %%
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from glob import glob

plt.rcParams["figure.dpi"] = 300

# %%
files = glob(r".\calibration\*.csv")
files
# %%
dry_air_OG = pd.read_csv(files[0], header=0, skiprows=4)  # dry air
atmosphere_OG = pd.read_csv(files[1], header=0, skiprows=4)  # open to atmosphere
wet_air_OG = pd.read_csv(files[2], header=0, skiprows=4)  # wet air
wet_N2_OG = pd.read_csv(files[4], header=0, skiprows=4)  # wet N2
dry_N2_OG = pd.read_csv(files[5], header=0, skiprows=4)  # wet N2
df_OGx = [dry_air_OG, atmosphere_OG, wet_air_OG, wet_N2_OG, dry_N2_OG]

# %%
fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, sharex=True, figsize=(5, 10))
plt.subplots_adjust(hspace=0.10)
ax5.set_xlabel("Time (s)")
ax1.set_ylabel("Oxy1 Voltage (V)")
ax2.set_ylabel("Oxy1 Temp ($^{\circ}$C)")
ax3.set_ylabel("BME Temp ($^{\circ}$C)")
ax4.set_ylabel("BME Humidity (%)")
ax5.set_ylabel("BME Pressure (kPa)")
for ax in [ax1, ax2, ax3, ax4, ax5]:
    ax.ticklabel_format(useOffset=False)
ax2.set_ylim(21.75, 22.75)
ax3.set_ylim(21, 22)
colors = ["#a6c5f7", "#347bed", "#0e449c", "#1d8012", "#008a7a"]
labels = ["dry air", "atm", "over water", "dry N2", "wet N2"]

for i, (df, color, label) in enumerate(zip(df_OGx, colors, labels)):
    df["elapsed time (s)"] = df["time (s)"] - df["time (s)"].iloc[0]
    if i in [0, 1, 2, 3, 4]:
        ax1.plot(
            df["elapsed time (s)"], df["oxy2_voltage (V)"], color=color, label=label
        )
        ax2.plot(df["elapsed time (s)"], df["oxy2_temp (C)"], color=color, label=label)
        ax3.plot(df["elapsed time (s)"], df["bme_temp (C)"], color=color, label=label)
        ax4.plot(df["elapsed time (s)"], df["humidity (%)"], color=color, label=label)
        ax5.plot(df["elapsed time (s)"], df["pressure (kPa)"], color=color, label=label)
# %%
# select 200 samples from each file
def select_samples(df, n=200):
    df = df.iloc[-200:].copy()
    df.reset_index(drop=True, inplace=True)
    df["elapsed time (s)"] = df["time (s)"] - df["time (s)"].iloc[0]
    return df


# %%
dry_air = select_samples(dry_air_OG)
atmosphere = select_samples(atmosphere_OG)
wet_air = select_samples(wet_air_OG)
wet_N2 = select_samples(wet_N2_OG)
dry_N2 = select_samples(dry_N2_OG)
dfx = [dry_air, atmosphere, wet_air, wet_N2, dry_N2]
# %%
fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, sharex=True, figsize=(5, 10))
plt.subplots_adjust(hspace=0.10)
ax5.set_xlabel("Time (s)")
ax1.set_ylabel("Oxy1 Voltage (V)")
ax2.set_ylabel("Oxy1 Temp ($^{\circ}$C)")
ax3.set_ylabel("BME Temp ($^{\circ}$C)")
ax4.set_ylabel("BME Humidity (%)")
ax5.set_ylabel("BME Pressure (kPa)")
for ax in [ax1, ax2, ax3, ax4, ax5]:
    ax.ticklabel_format(useOffset=False)
# ax2.set_ylim(21.75, 22.75)
ax3.set_ylim(21, 22)
colors = ["#a6c5f7", "#347bed", "#0e449c", "#1d8012", "#008a7a"]
labels = ["dry air", "atm", "over water", "dry N2", "wet N2"]

for i, (df, color, label) in enumerate(zip(dfx, colors, labels)):
    if i in [0, 2, 3, 4]:
        df["elapsed time (s)"] = df["time (s)"] - df["time (s)"].iloc[0]
        ax1.plot(
            df["elapsed time (s)"], df["oxy2_voltage (V)"], color=color, label=label
        )
        ax2.plot(df["elapsed time (s)"], df["oxy2_temp (C)"], color=color, label=label)
        ax3.plot(df["elapsed time (s)"], df["bme_temp (C)"], color=color, label=label)
        ax4.plot(df["elapsed time (s)"], df["humidity (%)"], color=color, label=label)
        ax5.plot(df["elapsed time (s)"], df["pressure (kPa)"], color=color, label=label)

# %%
# to keep things moving, for now let's just use the dry air and N2 measurements (as suggested by Apogee)
# and use averages over the selected 200 points

# Dry Calibration:
oxy1_Vc = np.mean(dry_air["oxy1_voltage (V)"])
oxy1_Tc = np.mean(dry_air["oxy1_temp (C)"])
oxy2_Vc = np.mean(dry_air["oxy2_voltage (V)"])
oxy2_Tc = np.mean(dry_air["oxy2_temp (C)"])
Pc = np.mean(dry_air["pressure (kPa)"])
Hc = np.mean(dry_air["humidity (%)"])

oxy1_V0 = np.mean(dry_N2["oxy1_voltage (V)"])
oxy1_T0 = np.mean(dry_N2["oxy1_temp (C)"])
oxy2_V0 = np.mean(dry_N2["oxy2_voltage (V)"])
oxy2_T0 = np.mean(dry_N2["oxy2_temp (C)"])
P0 = np.mean(dry_N2["pressure (kPa)"])
H0 = np.mean(dry_N2["humidity (%)"])

print("DRY CALIBRATION: \n")
print(
    f"\tOXY1:\n\t\tVcal: {oxy1_Vc}\n\t\tV0: {oxy1_V0}\n\t\tPcal: {Pc}\n\t\tRH: {Hc}\n\t\tTcal: {oxy1_Tc}"
)
print()
print(
    f"\tOXY2:\n\t\tVcal: {oxy2_Vc}\n\t\tV0: {oxy2_V0}\n\t\tPcal: {Pc}\n\t\tRH: {Hc}\n\t\tTcal: {oxy2_Tc}"
)
#%%
# Wet Calibration:
oxy1_Vc = np.mean(wet_air["oxy1_voltage (V)"])
oxy1_Tc = np.mean(wet_air["oxy1_temp (C)"])
oxy2_Vc = np.mean(wet_air["oxy2_voltage (V)"])
oxy2_Tc = np.mean(wet_air["oxy2_temp (C)"])
Pc = np.mean(wet_air["pressure (kPa)"])
Hc = np.mean(wet_air["humidity (%)"])

oxy1_V0 = np.mean(wet_N2["oxy1_voltage (V)"])
oxy1_T0 = np.mean(wet_N2["oxy1_temp (C)"])
oxy2_V0 = np.mean(wet_N2["oxy2_voltage (V)"])
oxy2_T0 = np.mean(wet_N2["oxy2_temp (C)"])
P0 = np.mean(wet_N2["pressure (kPa)"])
H0 = np.mean(wet_N2["humidity (%)"])

print("WET CALIBRATION: \n")
print(
    f"\tOXY1:\n\t\tVcal: {oxy1_Vc}\n\t\tV0: {oxy1_V0}\n\t\tPcal: {Pc}\n\t\tRH: {Hc}\n\t\tTcal: {oxy1_Tc}"
)
print()
print(
    f"\tOXY2:\n\t\tVcal: {oxy2_Vc}\n\t\tV0: {oxy2_V0}\n\t\tPcal: {Pc}\n\t\tRH: {Hc}\n\t\tTcal: {oxy2_Tc}"
)


# %%
# checking agreement on temperature readings between two sensors
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(5, 5))
ax1.plot(dry_air["elapsed time (s)"], dry_air["oxy1_temp (C)"], color="r")
ax1.plot(dry_air["elapsed time (s)"], dry_air["oxy2_temp (C)"], color="b")
diff = dry_air["oxy2_temp (C)"] - dry_air["oxy1_temp (C)"]
ax2.plot(dry_air["elapsed time (s)"], diff, color="k")
# %%
# calculate average difference in temperature readings between oxy1 and oxy2
for df in dfx:
    df["diff"] = df["oxy2_temp (C)"] - df["oxy1_temp (C)"]
for df in df_OGx:
    df["diff"] = df["oxy2_temp (C)"] - df["oxy1_temp (C)"]

# %%
fig, ax = plt.subplots()
for df in df_OGx:
    ax.plot(df["elapsed time (s)"], df["diff"])
# %%
