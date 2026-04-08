import HC_SR04
import time

Vorne = HC_SR04.HC_SR04(4, 25)
links = HC_SR04.HC_SR04(20, 21)
rechts = HC_SR04.HC_SR04(19, 16)

while True:
    print("Vorne: ", Vorne.measure())
    print("Links: ", links.measure())
    print("Rechts: ", rechts.measure())
    time.sleep(3)