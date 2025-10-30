1. Setup env vars
  get_idf

2. Build the project
  idf.py build

3. Flash to device
  idf.py -p <port> flash

4. See console logs
  idf.py -p <port> monitor

Notes:

- To flash and see console logs
  idf.py -p <port> flash monitor

- To get list of the ports (on macos)
  ls /dev/cu.*

- To clean the build
  idf.py clean
