world_xml = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="robotanik_sera">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
"""

# --- DIŞ DUVARLAR ---
# Kalınlık: 0.2m (20 cm), Yükseklik: 2m
duvarlar = [
    {"name": "sol_duvar", "pose": "0 25 1 0 0 0", "size": "0.2 50 2"},
    {"name": "sag_duvar", "pose": "11 25 1 0 0 0", "size": "0.2 50 2"},
    {"name": "alt_duvar", "pose": "5.5 0 1 0 0 0", "size": "11 0.2 2"},
    {"name": "ust_duvar", "pose": "5.5 50 1 0 0 0", "size": "11 0.2 2"}
]

for d in duvarlar:
    world_xml += f"""
    <model name="{d['name']}">
      <static>true</static>
      <link name="link">
        <pose>{d['pose']}</pose>
        <collision name="collision"><geometry><box><size>{d['size']}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{d['size']}</size></box></geometry>
          <material><script><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
    """

# --- İÇ ENGELLER (BİTKİ SIRALARI) ---
merkez_x_noktalari = [1.5, 3.5, 5.5, 7.5, 9.5]
for x in merkez_x_noktalari:
    world_xml += f"""
    <model name="bitki_sirasi_{x}">
      <static>true</static>
      <link name="link">
        <pose>{x} 25 0.5 0 0 0</pose>
        <collision name="collision"><geometry><box><size>1 46 1</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>1 46 1</size></box></geometry>
          <material><script><name>Gazebo/Green</name></script></material>
        </visual>
      </link>
    </model>
    """

world_xml += """
  </world>
</sdf>
"""

with open("robotanik_sera.world", "w") as f:
    f.write(world_xml)
print("Kapalı çevreli Gazebo 3D dünyası başarıyla yaratıldı!")
