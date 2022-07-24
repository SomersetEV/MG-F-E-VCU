# MG F-E VCU
 Code and designs for MG F electric conversion control board
 
 Hardware:
 Based on a teensy 3.6 board. The pcb can be made by JLCpcb using the files attched. 
 Enclosure is a 56 pin automotive enclosure, can be bought from Aliexpress here: https://www.aliexpress.com/item/32857771975.html?spm=a2g0s.12269583.0.0.1221609agr7UAj
 
 There is a design error where the 5v track has merged with a through hole for mounting. Tape over this throughhole and and don't use it as it will short out the 5v line.
 
 While the board was initally designed to use the Prius Gen 2 inverter for charging, it can be modified and the 12v outputs used for other signals. Canbus in the most recent updates now enable the use of an Outlander Charger.
 
