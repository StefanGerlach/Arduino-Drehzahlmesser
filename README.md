# Arduino-Drehzahlmesser
Ein Drehzahlmesser für KFZ oder Krad, basierend auf der Arduino Uno Plattform und 2,4" TFT Display.


Dies ist ein kleines Projekt zur Realisierung eines Drehzahlmessers. Grundlage der Messung ist eine Zeitmessung zwischen Impulsen, die via Interrupt erfasst werden. Als Plattform kommt das Arduino Uno Board zur Anwendung [https://www.arduino.cc/en/Main/arduinoBoardUno].

Der ATmega328P, getaketet auf 16 MHz stellt genug GPIO Pins und Funktionen zur Verfügung, um sowohl die echtzeitfähige Messung der Zeit zwischen Impulsen auf dem Interrupt GPIO-Pins, als auch die Darstellung des gemessenen Wertes auf einem TFT-Display zu realisieren.

Die aktuelle Version beinhaltet die rudimentären Funktionen - noch ohne Schaltplan - und wurde mit einem RaspberryPI 3 als Signalgenerator getestet.
