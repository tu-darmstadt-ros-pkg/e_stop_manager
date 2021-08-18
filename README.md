# e_stop_manager
The E-Stop Manager merges multiple E-Stop sources.

### Config file
* **e_stop_list_topic:** topic name, here the lists of e-stop names and values are published
* **e_stop_topic:** topic name, here the accumulated e-stop value is published (if any of the values is true, this value is true)
* **init_e_stop_value:** bool, all e-stop values are set to this value on init
* **e_stop_list:** list of all e-stop names


### Set e-stop value
The sources can make service calls on service "/e_stop_manager/set_e_stop" to set the e-stop value. The service has the following parameters:

* **Request**:
    * **string e_stop_name**: Name of e-stop, needs to be in e_stop_list in config file.
    * **bool e_stop_value**: true, if e-stop is pressed, false otherwise.

* **Response**:
    * **int8 result**: Result of service call (see below)

* **Result constants**:
    * int8 SUCCESS=0
    * int8 FAILURE=1
    * int8 INVALID_ESTOP_NAME=2