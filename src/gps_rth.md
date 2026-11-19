# 🧭 openXsensor RTH & Orbit-Steuerung – Technische Dokumentation

Diese Dokumentation erklärt alle Variablen und den genauen Rechenweg der RTH- und Orbit-Logik (`calcOrbitRTH`) für den RP2040.

---

## 1. Das Variablen-Lexikon (Nachschlagewerk)

### A) Einstellparameter (Terminal-Befehle & Anpassung)
Diese Werte werden über das serielle Terminal konfiguriert (`RTS=...` und `RTD=...`).

| Variable im Code | Terminal-Befehl | Einheit (Eingabe) | Bedeutung & Funktion | Richtwert Auto | Richtwert Segler |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `active_min_dist` | `RTD` | Meter (`m`) | Gewünschter Kreisradius um den Startpunkt (`target_radius_m`). | `3` bis `5` | `60` bis `80` |
| `active_min_speed` | `RTS` | `km/h` (`0–255`) | Mindestgeschwindigkeit, ab der die Lenkung aktiv wird. | `1` bis `3` | `5` bis `12` |
| `use_orbit_mode` | *(Config)* | `true` / `false` | `true` = Kreisbahn (`calcOrbitRTH`), `false` = Direktanflug (`calcDirectRTH`). | `true` | `true` |

### B) Positions- & Bewegungsdaten (Eingangswerte vom GPS)
Diese Werte liefert das GPS-Modul direkt in den Code.

| Variable im Code | Einheit | Bedeutung & Funktion |
| :--- | :--- | :--- |
| `fields[LATITUDE].value` | `1e-7 Grad` | Aktueller Breitengrad (Nord/Süd) direkt vom GPS. |
| `fields[LONGITUDE].value` | `1e-7 Grad` | Aktueller Längengrad (Ost/West) direkt vom GPS. |
| `last_lat` / `last_lon` | `1e-7 Grad` | Gespeicherte Koordinaten vom vorherigen gültigen Schritt (um die Fahrtrichtung zu erkennen). |
| `home_lat` / `home_lon` | `1e-7 Grad` | Fest gespeicherte Koordinaten des Startpunktes (Home). |
| `lat_scale` | Faktor (`cosf`) | Automatische Breitengrad-Korrektur (gleicht die Erdkrümmung weltweit aus). |
| `current_speed_kmh` | `km/h` | Aktuelle Geschwindigkeit über Grund. |
| `step_dist_m` | Meter (`m`) | Zurückgelegte Strecke seit dem letzten gültigen GPS-Schritt (Filter gegen Stand-Drift). |

### C) Berechnete Navigations- & Steuerwerte (Zwischen- und Ausgangswerte)
Diese Werte berechnet der RP2040 in jedem Durchlauf neu.

| Variable im Code | Einheit / Bereich | Bedeutung & Funktion |
| :--- | :--- | :--- |
| `distance_m` | Meter (`m`) | Aktuelle Luftlinien-Entfernung zwischen Modell und Startpunkt. |
| `bearing_from_home` | `0°` bis `360°` | Himmelsrichtung vom Startpunkt zum Modell (0° = Nord, 90° = Ost, 180° = Süd, 270° = West). |
| `bearing_to_home` | `0°` bis `360°` | Direkte Richtung vom Modell zurück zum Startpunkt (`bearing_from_home + 180°`). |
| `track_deg` | `0°` bis `360°` | Tatsächliche aktuelle Bewegungsrichtung (Ist-Kurs) des Modells. |
| `tangent_offset_deg` | `0°` bis `135°` | Geometrischer Tangentenwinkel (`asinf(R / D)`), der das Modell exakt an den Kreisrand führt. |
| `target_heading` | `0°` bis `360°` | Endgültiger Zielkurs (Himmelsrichtung), in den die Nase zeigen soll. |
| `turn_angle` | `-180°` bis `+180°` | Benötigter Lenkwinkel (Differenz zwischen Zielkurs und aktueller Bewegungsrichtung). |
| `target_cmd` | `0` bis `10000` | Ungefilterter Roh-Steuerwert (`5000` = Neutralstellung / 50.00 im Sender). |
| `current_heading_cmd` | `0` bis `10000` | Geglätteter finaler Steuerwert (70 % neu / 30 % alt) für das Telemetrie-Feld `HEADING`. |

---

## 2. Das Ablauf-Schema (Wenn-Dann-Logik)

Bei jedem neuen Positions-Paket vom GPS durchläuft der Code exakt diese fünf Schritte:

### Schritt 1: Sicherheits-Check (Darf gerechnet werden?)
* **Prüfung:** Ist der Startpunkt sicher gespeichert (`home_is_locked == true`)?
  * **NEIN:** Es wird nichts berechnet, Ausgabe bleibt auf `5000` (Mitte).
  * **JA:** Weiter zu Schritt 2.

### Schritt 2: Nahbereichs- & Pilotenschutz prüfen
* **Direkt-Modus (`!use_orbit_mode`):** Ist `distance_m <= active_min_dist`, wird sofort `5000` (Neutral) ausgegeben.
* **Orbit-Modus (`calcOrbitRTH`):** Ist `distance_m < 10.0f` (Modell fliegt direkt über dem Piloten), gibt die Funktion sofort `5000` zurück, um hektische Manöver im Zenit zu verhindern. *(Hinweis: Für enge RC-Car-Tests unter 10 m diesen Wert im Code auf `1.5f` senken.)*

### Schritt 3: Bewegungs- & Schrittweiten-Filter prüfen
* **Prüfung:** Ist `current_speed_kmh > active_min_speed` **UND** beträgt die zurückgelegte Strecke seit dem letzten Punkt mindestens `3,5 Meter` (`step_dist_m >= 3.5f`)?
  * **NEIN (Stillstand / zu langsam):** Das GPS-Rauschen würde falsche Winkel erzeugen. Der Code hält den letzten gültigen Wert (`current_heading_cmd`) fest.
  * **JA (Fahrt aktiv):** Weiter zu Schritt 4.

### Schritt 4: Tangenten-Kurs berechnen (`calcOrbitRTH` – Ausführlich erklärt)

Stell dir vor, du stehst am Startpunkt (Home) und schaust nach **Norden (`0°`)**. Genau dort draußen, in **10 Metern Entfernung (`distance_m`)**, fährt dein Auto geradewegs weiter nach **Norden (`0°`)** von dir weg. Dein Zielkreis (`target_radius_m`) hat einen Radius von **5 Metern**.

### RTH-Steuerlogik & Gegenwind-Verhalten (Orbit / Kreisen)

* **Startzustand (vor Home-Lock & am Boden):** 
  * Bis `home_is_locked` aktiv ist (und im Stand unterhalb der Mindestgeschwindigkeit `min_speed_cms`), wird `GPS_custom_steering` permanent auf **`5000`** (Neutralstellung) gehalten.
* **Flug außerhalb des Kreises (`distance_m > target_orbit_r`):** 
  * Es wird die geometrische Tangente über `asinf(target_orbit_r / distance_m)` berechnet und auf maximal **`80°`** begrenzt (10° Vorhalt nach innen kompensieren Trägheit und den 1-Hz-GPS-Takt).
* **Flug innerhalb des Kreises (`distance_m <= target_orbit_r`):** 
  * Der Einlenkwinkel wird linear von **`80°` bis `135°`** nach außen erhöht, um das Modell sanft auf den Kreisrand zurückzudrücken.
* **Verhalten bei starkem Gegenwind (`current_ubx_speed <= min_speed_cms`):** 
  * Fällt die Grundgeschwindigkeit (z. B. im Segler bei starkem Gegenwind) unter die eingestellte Mindestgeschwindigkeit `RTS`, greift der `else`-Zweig. 
  * **Wichtig:** Der Wert wird **nicht** auf `5000` zurückgesetzt (was das Ruder neutral stellen und den Segler vom Wind abtreiben würde), sondern der **letzte gültige Lenkwinkel** bleibt eingefroren, bis wieder Fahrt aufgenommen wird.


So rechnet der Code in vier Teilschritten aus, wohin das Auto lenken muss:

1. **Wo fährt das Auto gerade hin? (`track_deg`)**
   * Der Code vergleicht die alte Position (`last_lat/lon`) mit der neuen Position (`fields[LATITUDE/LONGITUDE]`).
   * *In unserem Beispiel:* Das Auto fährt nach Norden, also ist **`track_deg = 0°`**.

2. **Wo liegt der Startpunkt aus Sicht des Autos? (`bearing_to_home`)**
   * Du schaust vom Startpunkt nach Norden (`bearing_from_home = 0°`) zum Auto.
   * Vom Auto aus liegt der Startpunkt also genau in der Gegenrichtung (`0° + 180°`): **`bearing_to_home = 180°` (Süden)**.

3. **Wie schräg muss das Auto zielen, um den Kreisrand zu treffen? (`tangent_offset_deg`)**
   * Da das Auto außerhalb des Kreises ist (`10 m > 5 m`), berechnet der Code über den Arkussinus (`asinf(5 / 10)`) den exakten Tangentenwinkel zum rechten Kreisrand: **`tangent_offset_deg = 30°`**.
   * *Wichtig – Aktive Nachregelung am Kreisrand (`80°`-Vorhalt):*
     * Am Kreisrand (`5 m`) wird der Tangentenwinkel auf **`80°`** (statt `90°`) begrenzt. Das ist **kein starrer Servowert**, sondern ein **geometrischer Vorhaltewinkel von 10° nach innen** für die Soll-Kurs-Berechnung.
     * Da das System in jedem GPS-Schritt den Ist-Kurs (`track_deg`) mit dem neuen Soll-Kurs (`target_heading`) vergleicht, bleibt durch diese `10°` Kurs-Vorlage dauerhaft eine sanfte Kurven-Nachregelung aktiv.
     * Treibt Wind oder Fliehkraft das Modell dennoch leicht nach außen (`distance_m > 5 m`), sinkt der Winkel `asinf(R / D)` sofort weiter ab (z. B. auf `70°`), und der Code lenkt das Modell aktiv zurück auf die Kreislinie.
     * *Innerhalb* des Kreises (`< 5 m`) drückt der Code das Auto mit `80°` bis `135°` proportional nach außen zurück auf den Kreis.

4. **Welcher endgültige Zielkurs ergibt sich daraus? (`target_heading`)**
   * Für einen Kreis im Uhrzeigersinn zieht der Code den Tangentenwinkel von der Heim-Peilung ab:
     `180° (Süden) - 30° = 150°` (**Süd-Süd-Ost**).
   * *Das Ergebnis:* Das Auto steuert jetzt exakt auf **`150°`** zu. Dadurch fährt es auf einer geraden Linie genau an den äußeren Rand deines 5-Meter-Kreises und schwenkt dort nahtlos auf die Kreisbahn ein.

### Schritt 5: Lenkwinkel ermitteln, glätten & für EdgeTX umrechnen (Ausführlich erklärt)

Jetzt weiß der Code, wohin das Auto fährt (`track_deg = 0°`) und wohin es fahren *soll* (`target_heading = 150°`). Daraus erzeugt er das geglättete Steuersignal für das Servo:

1. **Wie weit muss das Lenkrad eingeschlagen werden? (`turn_angle`)**
   * Der Code berechnet die Differenz: `Zielkurs - Ist-Kurs` (`target_heading - track_deg`).
   * *In unserem Beispiel:* `150° - 0° = +150°`. Das Auto muss also eine **150°-Rechtskurve** einleiten.
   * **Der 180°-Überlauf-Schutz:** Falls bei einer Rechnung z. B. `+270°` herauskommt, zieht der Code `360°` ab (`-90°`), damit das Auto immer den kürzesten Weg (hier 90° links statt 270° rechts) lenkt.

2. **Umrechnung in die Fernsteuerungs-Skala (`target_cmd`)**
   * Deine Fernsteuerung (EdgeTX) erwartet für das Telemetriefeld `HEADING` einen Zahlenwert zwischen **`0`** und **`10000`** (im Sender wird das als `0.00` bis `100.00` angezeigt).
   * Die Formel `((turn_angle / 3.6f) + 50.0f) * 100.0f` übersetzt den Winkel exakt in diese Skala:
     * **`-180°`** (maximale Linkskurve) $\rightarrow$ `((-180 / 3.6) + 50) * 100` = **`0`**
     * **`0°`** (geradeaus, kein Fehler) $\rightarrow$ `((0 / 3.6) + 50) * 100` = **`5000`** (Mitte)
     * **`+150°`** (unser Beispiel) $\rightarrow$ `((150 / 3.6) + 50) * 100` = **`9167`** (starker Rechtsausschlag)
     * **`+180°`** (maximale Rechtskurve) $\rightarrow$ `((180 / 3.6) + 50) * 100` = **`10000`**

3. **Weiche Glättung & Ausgabe an den Sender (`current_heading_cmd`)**
   * Um harte Sprünge des 1-Hz-GPS abzufedern, mischt der Code den neuen Wert zu 70 % mit dem vorherigen Wert zu 30 %: `(current_heading_cmd * 3 + target_cmd * 7) / 10`.
   * Der fertige Wert wird über `setField(fields[HEADING], current_heading_cmd)` an den Sender geschickt. Dein Input und Mischer in EdgeTX wandeln diesen Wert direkt in den passenden Servoausschlag für die Lenkung oder das Seitenruder um.

---

## 3. Der Praxis-Flugablauf nach Aktivierung (4 Phasen)

Wenn das Modell weit draußen (z. B. bei 250 Metern, Soll-Radius `RTD = 80 m`) geradewegs wegfliegt und der **RTH-Schalter** umgelegt wird, durchläuft es automatisch diese vier Flugphasen:

### Phase 1: Aktivierung & Die Umkehrkurve
* **Auslöser:** Der RTH-Schalter an der Fernsteuerung wird umgelegt und schaltet das `HEADING`-Signal auf das Seitenruder (Gyro stabilisiert Quer und Höhe).
* **Berechnung:** Da das Modell weit draußen ist, berechnet der Arkussinus (`asinf(80 / 250)`) nur einen kleinen Tangentenwinkel von ca. **`18,6°`**. Der Zielkurs zeigt also fast direkt (`161,4°`) auf den rechten Rand des 80-Meter-Kreises.
* **Flugverhalten:** Der große Richtungsfehler (`turn_angle` nahe ±180°) erzeugt den vollen proportionalen Ruderausschlag. Das Modell leitet eine ruhige Umkehrkurve ein.

### Phase 2: Der direkte Heimflug zum Kreisrand
* **Kurs liegt an:** Nach der Umkehrkurve fliegt das Modell auf einer **schnurgeraden Tangenten-Linie** direkt auf den äußeren Rand des 80-Meter-Kreises zu (ohne weites Ausholen).
* **Ruder in Neutralstellung:** Da Ist-Kurs (`track_deg`) und Soll-Kurs (`target_heading`) jetzt übereinstimmen, wird der Lenkwinkel zu `0°`. Der RP2040 sendet **`5000` (50.00)** und das Seitenruder kehrt in die Mitte zurück.

### Phase 3: Das tangentiale Einschwenken auf den Zielkreis
* **Annäherung:** Kurz vor dem 80-Meter-Kreis öffnet sich der Tangentenwinkel (`asinf`) automatisch bis auf **`80°`**.
* **Weicher Übergang:** Das Seitenruder dreht das Modell nahtlos nach außen auf die Kreisbahn ein, ohne über das Ziel hinauszuschießen.

### Phase 4: Der endlose Wartekreis (Orbit)
* **Gleichgewicht:** Auf dem 80-Meter-Kreis hält der **`80°`-Vorhalt** (`10°` nach innen gegen Fliehkraft und 1-Hz-Trägheit) das Modell exakt auf der Kreisbahn im Uhrzeigersinn, bis der RTH-Schalter wieder deaktiviert wird.

---

## 4. EdgeTX-Konfiguration (Input / Geber & Mischer)

Die Signalaufbereitung erfolgt in EdgeTX sauber getrennt über einen eigenen **Input (Geber)** und eine anschließende **Mischer-Zeile**:

### A) Input (Geber `RTH` anlegen)
Hier wird der reine Telemetriewert (`0` bis `100`, Mitte `50`) in ein vollwertiges symmetrisches Steuersignal (`-100 %` bis `+100 %`, Mitte `0 %`) umgewandelt.

| Parameter im Input | Einstellung | Bedeutung & Funktion |
| :--- | :--- | :--- |
| **Input Name** | `RTH` | Eigener virtueller Steuergeber für die Rückkehrfunktion. |
| **Quelle (Source)** | `Hdg` (bzw. `HEADING`) | Telemetriefeld vom RP2040. |
| **Gewichtung (Weight)** | **`100 %`** *(oder `200 %` je nach Sensor-Skalierung)* | Nutzt den vollen Regelbereich des Sensors. |
| **Offset** | **`-50 %`** | Verschiebt den Telemetrie-Neutralwert (`50.00`) exakt auf `0 %` Gebermitte. |

### B) Mischer (Mixes auf Lenkung / Seitenruder)
Hier wird der fertige Geber `RTH` über den Schalter auf das bestehende Rudersignal aufaddiert.

| Parameter im Mischer | RC-Car | Segler (ASK 14) | Bedeutung & Funktion |
| :--- | :--- | :--- | :--- |
| **Quelle (Source)** | `I:RTH` | `I:RTH` | Der unter A) angelegte virtuelle RTH-Geber. |
| **Gewichtung (Weight)** | **`200 %`** | **`80 %`** *(ggf. `-80 %`)* | Bestimmt die maximale Lenkstärke (Auto braucht voller Einschlag, Segler weiche Kurven). |
| **Schalter (Switch)** | z. B. `SA↓` | z. B. `SA↓` | Aktiviert und deaktiviert die automatische Steuerung. |
| **Multiplex** | **`Addieren` (Add)** | **`Addieren` (Add)** | Überlagert das manuelle Knüppelsignal (jederzeit übersteuerbar). |

### Wirkrichtungstest vor dem Einsatz
1. **Home-Lock abwarten:** Modell einschalten und warten, bis der Startpunkt gespeichert ist.
2. **Bewegung simulieren:** Weiter als `RTD` weggehen, zügig vom Startpunkt weglaufen (`Speed > RTS`) und den **RTH-Schalter** umlegen.
3. **Ruder prüfen:** Das Seitenruder (bzw. die Lenkung) muss in eine Kurve zurück zum Startpunkt einlenken. Bei falscher Richtung das Vorzeichen im Mischer (z. B. von `+80 %` auf `-80 %`) umkehren.
