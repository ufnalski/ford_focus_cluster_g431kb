# Ford Focus Mk3 (2011+) instrument panel cluster hacking (STM32G431KB)
A similar story as for the [Mustang](https://github.com/ufnalski/ford_mustang_cluster_h503rb). Let me therefore not to retell the whole story here.

![Ford Focus Mk3 IPC pinout](/Assets/Images/ford_focus_mk3_ipc_pinout.PNG)
![Ford Focus Mk3 IPC socket](/Assets/Images/ford_focus_mk3_ipc_socket.jpg)
![Ford Focus Mk3 IPC part number](/Assets/Images/ford_focus_mk3_ipc_part_number.jpg)
![Ford Focus Mk3 menu switch back](/Assets/Images/ford_focus_mk3_menu_switch_back.jpg)
![Ford Focus Mk3 menu switch](/Assets/Images/ford_focus_mk3_menu_switch.PNG)
![Ford Focus Mk3 resistor ladder](/Assets/Images/menu_resistor_ladder.PNG)
![Ford Focus Mk3 PulseView CAN decoder](/Assets/Images/pulseview_can_decoder.PNG)
![Ford Focus Mk3 IPC in action](/Assets/Images/ford_focus_mk3_ipc_in_action.jpg)
![Ford Focus Mk3 replacement part](/Assets/Images/resistor_ladder_switch.jpg)

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Wiring diagrams
* [Electrical circuit schematics Ford C346 MY2011](https://navody.ford-focus.cz/Ford_Focus_manualy/mk3/FF3_el_schemata.pdf)

# Other Focus Mk3 reverse engineers
* [Ford Focus MK3 Cluster Controller w/ ESP32 or python-can](https://github.com/gizmo87898/FordFocusMK3_Cluster_BeamNG) (gizmo87898)
* [CAN simulator for Ford Focus mk3 IPC](https://github.com/bigunclemax/FocusIPCCtrl) (bigunclemax)

# Decoder
* [PulseView](https://sigrok.org/wiki/PulseView) (sigrok)

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_otwarty_we/Dzien_Otwarty_WE_2024_Control_Engineering_for_Hobbyists.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [Max Imagination](https://www.youtube.com/@MaxImagination), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Automotive hacking - do try this at home :sunglasses:

200+ challenges to start from: [Control Engineering for Hobbyists](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf) at the Warsaw University of Technology.

Stay tuned!
