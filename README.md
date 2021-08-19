# e_stop_manager
The E-Stop Manager merges multiple E-Stop sources.

### Config file
* **e_stop_list:** list of all e-stop names and initial values, each entry must have an entry "name" and "value".


### Set e-stop value
The sources can make service calls on service "/e_stop_manager/set_e_stop" to set the e-stop value. The service has the following parameters:

* **Request**:
    * **string name**: Name of e-stop, needs to be in e_stop_list in config file.
    * **bool value**: true, if e-stop is pressed, false otherwise.

* **Response**:
    * **int8 result**: Result of service call (see below)

* **Result constants**:
    * int8 SUCCESS=0
    * int8 FAILURE=1
    * int8 INVALID_ESTOP_NAME=2
