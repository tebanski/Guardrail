# Guardrail
Guardrail vehicle IIoT project

To use, flash all three files onto your M5STACK and restart.

From REPL, do:

import driver
driver.init()

You can disable initializing the GPS with:

driver.init(enabe_gps=False)
