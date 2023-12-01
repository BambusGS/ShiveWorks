# © Jakub Jandus 2023
# Overseer class for the ShiveWorks project
# This class is responsible for the communication with the segments and the movement of the segments

import paho.mqtt.client as mqtt
import csv
import time
from datetime import datetime, timezone
from struct import pack, unpack
# from sys import getsizeof

# ---TODO--- #


# ---------------------- MQTT Setup----------------------#
# callback function for when a message is received and filter according to topic

segment_count = 40  # number of segments
overseerCommandPath = "ShiveWorks/overseer/command"
overseerReturnPath = "ShiveWorks/overseer/return"

# expanded into ShiveWorks/segment/segmentID/command further below
segmentPath = "ShiveWorks/segment"

# local path to the folder containing the actuation data in .csv format
actuationDataPath = "./PropertyControlFunctions/Actuation_data"

# milliseconds of countdown time after a "start" command is issued; do not set to less than 3000
countdown_ms = 3200

# local broker address, if the server runs on the same PC as the client never change it
broker = "127.0.0.1"
port = 1884
client = mqtt.Client("Master PC")


def main():  # main method where the MQTT client is connected
    # try to connect to the broker, if not successful wait 5 seconds and try again
    while True:
        try:
            client.connect(broker, port)
            break
        except:
            print("Connection to broker failed, retrying in 5 seconds...")
            time.sleep(5)

    client.subscribe(overseerReturnPath, 1)
    client.loop_start()
    client.on_message = on_message
    loadSegmentsID()


# --- MQTT Received Message Callback functions ---#

latestOverseerReturnMessage = ""  # global storage of the latest message received
# list of the status of all segments ordered by segment number
statusList = [None]*segment_count


def on_message(client, userdata, message):
    global latestOverseerReturnMessage  # use the global variable
    print("Message received: " + str(message.payload.decode("utf-8")))
    topic = message.topic
    msgPayload = message.payload.decode("utf-8")

    # filter only the return messages
    if (topic == overseerReturnPath):
        latestOverseerReturnMessage = str(msgPayload)

    # filter only the status messages
    elif (topic.startswith(segmentPath) and topic.endswith("status")):
        # get the segment ID from the topic -> convert it to the segment number
        segment_no = getSegmentNumber(topic.split("/")[2])
        # get the status from the message
        status = str(msgPayload)
        # store the status in the list
        statusList[segment_no - 1] = status

        # special case to confirm pairing
        #   if a segment ID is assigned and a message "Connected" is received, send back an "ack"
        if status == "Connected" and getSegmentID(segment_no) != "Null":
            segmentAck(segment_no)

    # filter only the data return messages
    elif (topic.startswith(segmentPath) and topic.endswith("data")):
        # get the segment ID from the topic -> convert it to the segment number
        segment_no = getSegmentNumber(topic.split("/")[2])
        # get the status from the message
        data = msgPayload
        # store the data in the return list
        # statusList[segment_no - 1] = status


# ---------------------- Dealing with segment identification ----------------------#

segments_ID = []  # an ordered list of all the segments' ID's
# this list is 100 elements long, with each element being a string
filePath = "segmentsID.csv"


# get the ID of a segment from the list and return it as a string (Null if not found or empty)
def getSegmentID(segment_no):
    try:
        if int(segment_no) > segment_count or int(segment_no) < 1:
            raise IndexError
        else:
            return segments_ID[int(segment_no) - 1]

    except IndexError:
        print("Segment number out of range")
        return "Null"


# get the segment number from the ID and return it as an integer (-1 if not found)
def getSegmentNumber(segmentID):
    for i in range(len(segments_ID)):
        if segments_ID[i] == segmentID:
            return int(i + 1)
    return int(-1)


def addSegmentID(segment_no):  # add a segment ID to the list and save it to a .csv file
    # check if there is a segment available for assignment
    if latestOverseerReturnMessage.startswith('pairing'):

        # check if the segment number is valid (between 0 and maxSegmentNumber)
        if int(segment_no) < 0 and int(segment_no) > segment_count:
            print("Invalid segment number")
            return False
        else:
            ID_str = latestOverseerReturnMessage.split("::")[1]
            segments_ID[int(segment_no) - 1] = ID_str
            saveSegmentsID()

            # subscribe to the status and data return of the new segment
            segmentSub(segment_no)

            # acknowledge to the segment that pairing was successful
            segmentAck(segment_no)

            # print segment number and id that has been added to the list
            print("Segment #{} has been added to the list with ID: {}".format(
                segment_no, ID_str))
            return True
    else:
        print("No segment available for assignment")
        return False


# remove a segment ID from the list and save it to a .csv file
def removeSegmentID(segment_no):
    # check if the segment number is valid
    if getSegmentID(segment_no) == "Null":
        print("Invalid segment number")
        return False
    else:
        segments_ID[int(segment_no) - 1] = "Null"
        saveSegmentsID()

        # unsubscribe from the status and data return of the segment
        segmentUnSub(segment_no)

        # print segment number and id that has been removed from the list
        print("Segment #{} has been removed from the list".format(segment_no))
        return True


def loadSegmentsID():   # load the segments ID from a .csv file
    try:
        segments_ID.clear()  # clear the list before loading
        with open(filePath, 'r') as file:
            reader = csv.reader(file, delimiter=' ')
            for row in reader:
                segments_ID.append(row[0])

        # subscribe to the status of all segments if they exist
        for i in range(1, len(segments_ID) + 1):
            if getSegmentID(i) != "Null":
                # subscribe to the status and data return of the new segment
                segmentSub(i)
        return True
    except Exception as e:
        print("Failed to load segments ID list: {}".format(e))
        return False


def saveSegmentsID():   # save the segments ID to a .csv file
    try:
        with open(filePath, 'w', newline='') as file:
            writer = csv.writer(file, delimiter=' ')
            for ID in segments_ID:
                writer.writerow([ID])
        return True
    except Exception as e:
        print("Failed to save segments ID list: {}".format(e))
        return False


def clearSegmentID(segment_no):  # clear a segment ID from the list
    return segmentCommand(segment_no, "restart") and removeSegmentID(segment_no)


def clearSegmentsID():  # clear the segments ID list
    segmentMasterCommand("stop")
    segments_ID.clear()
    for i in range(segment_count):
        segments_ID.append("Null")
    saveSegmentsID()
    print("Segments ID list has been cleared, resetting program...")
    client.loop_stop()
    quit()

# ---------------------- Actuation Data Conversion ----------------------#


# a 2D array to hold the actuation time and value [[1000, 255], [1001, 254]]
# timestamps are stored as 16-bit unsigned integers, so the number shall be between 0 and 65535 only
# actuation value is an 8-bit unsigned integer, so the number shall be between 0 and 255 only
#   with the exception that a .csv value of -100 means skip this actuation line
actuationDataArray = []


# function to first load all the .csv file from the folder (if it exists) into an array while type-checking
#    writes the first and last value regardless of if it should be dropped
#    if no value is present at start (= start material value is -100), assume center position of 127
def loadSegmentData(segment_no):
    global actuationDataArray
    firstRow = [0, 0]
    lastRow = [0, 0]

    segmentDataPath = actuationDataPath + '/' + segment_no + ".csv"
    actuationDataArray.clear()

    try:
        with open(segmentDataPath, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            # go through each row and convert them to the [time, value] format
            row_index = 0
            for row in reader:
                if row_index == 0:  # save the first row
                    firstRow = row
                lastRow = row  # save the last row

                # convert the segment values
                row_clean = convertSegmentData(row)
                if row_clean:  # if the array not empty, append it
                    actuationDataArray.append(row_clean)

                row_index = row_index + 1

    except Exception as e:
        print("Failed to load segments data: {}".format(e))
        return False

    # convert the values to integers
    firstRow[0] = int(firstRow[0])
    firstRow[1] = int(firstRow[1])
    lastRow[0] = int(lastRow[0])
    lastRow[1] = int(lastRow[1])

    # print(firstRow)
    # print(lastRow)
    # print(actuationDataArray)

    # if the first row is value-empty, then write the default middle value
    if firstRow[1] == -100:
        firstRow[1] = 127
        actuationDataArray.insert(0, firstRow)

    # if the last timestamp of the array is not the same time as in the last row of the .csv file
    #    then copy the last actuation value at the final timestamp
    if actuationDataArray[-1][0] != lastRow[0]:
        lastTimestamp = lastRow[0]
        lastActuationData = actuationDataArray[-1][1]
        actuationDataArray.append([lastTimestamp, lastActuationData])

    # print("----")
    # print(firstRow)
    # print(lastRow)
    # print(actuationDataArray)

    return True


# function that checks the timestamp and value validity
def convertSegmentData(CSV_row):
    global actuationDataArray, firstRow, lastRow
    CSV_row_clean = []

    timestamp = int(CSV_row[0])
    if timestamp < 0 or timestamp > 65535:
        raise ValueError("Invalid timestamp: {}".format(timestamp))

    value = int(CSV_row[1])

    if value == -100:
        return CSV_row_clean

    if value > 255 or value < 0:
        raise ValueError("Actuation value is out of range: {}".format(value))

    CSV_row_clean.append(timestamp)
    CSV_row_clean.append(value)
    return CSV_row_clean


# function that takes the processed data and returns a struct formatted as binary uint16_t, uint8_t, ...
def packageSegmentData(segment_no):
    global actuationDataArray
    # https://docs.python.org/3.7/library/struct.html#struct.pack_into
    if loadSegmentData(segment_no):
        # formatting for the ESP32 shall be little-endian, 2-byte unsigned short, 1-byte unsigned char
        packed_binary = bytes()
        # packed_binary = pack('<' + 'HB' * len(actuationDataArray), *actuationDataArray[:][0], *actuationDataArray[:][1])

        for i in range(len(actuationDataArray)):
            packed_binary += pack('<' + 'HB',
                                  actuationDataArray[i][0], actuationDataArray[i][1])

        # print(packed_binary)
        # print(unpack('<' + 'HB' * len(actuationDataArray), packed_binary))
        # check that the packed binary is smaller than 64kB
        # print("Payload size: " + str(getsizeof(packed_binary)))
        return packed_binary
    # return packed byte struct
    return None


# ---------------------- Individual Segment Functions ----------------------#
# functions to simplify ESP32 messages

def segmentSub(segment_no):
    if getSegmentID(segment_no) != "Null":
        client.subscribe(segmentPathFn(segment_no, "status"), 1)
        client.subscribe(segmentPathFn(segment_no, "return"), 1)
        return True
    else:
        return False


def segmentUnSub(segment_no):
    if getSegmentID(segment_no) != "Null":
        client.unsubscribe(segmentPathFn(segment_no, "status"), 1)
        client.unsubscribe(segmentPathFn(segment_no, "return"), 1)
        return True
    else:
        return False


# sends a predefined command to an individual segment
def segmentCommand(segment_no, command):
    if getSegmentID(segment_no) != "Null":
        client.publish(segmentPathFn(segment_no, "command"), command, 1)
        return True
    else:
        return False


def segmentAck(segment_no):
    return segmentCommand(segment_no, "ack")


def segmentSendData(segment_no):
    # send data to specific segment
    data = packageSegmentData(segment_no)
    if getSegmentID(segment_no) != "Null" and data != None:
        # loads, processes, and converts actuation data
        client.publish(segmentPathFn(segment_no, "data"), data, 1)
        return True
    else:
        return False


def segmentMasterCommand(command):
    # command to the overseer master topic that goes to all segments
    client.publish(overseerCommandPath, command, 1)


# ---------------------#


def segmentPathFn(segment_no, whatToDo):
    # there are four topics for each segment: command, return, data, and status
    if whatToDo == "command" or whatToDo == "return" or whatToDo == "status" or whatToDo == "data":
        if getSegmentID(segment_no) != "Null":
            return segmentPath + "/" + getSegmentID(int(segment_no)) + "/" + whatToDo


def segment_reset(segment_no):  # reset a specific segment
    return segmentCommand(segment_no, "reset")


def segment_restart(segment_no):  # restart a specific segment
    return segmentCommand(segment_no, "restart")


def timesync_segment(segment_no):  # timesync a specific segment
    return segmentCommand(segment_no, "timesync")


def move_segment(segment_no, position):  # move a specific segment to a position
    # check that the position is within the range of the segment 0-255
    if int(position) > 255 or int(position) < 0:
        print("Invalid position")
        return False
    else:
        return segmentCommand(segment_no, "move::{}".format(position))


def get_segment_status(segment_no):  # get the status of a specific segment
    segmentCommand(segment_no, "status_report")
    time.sleep(0.25)  # wait for the status report to be published
    try:
        return statusList[int(segment_no) - 1]
    except IndexError:
        return ("Segment does not have a reported status yet")


# ---------------------- Manual Commands ----------------------#
if __name__ == "__main__":
    main()

while True:
    # an infinite loop that waits for a command to control the whole shive machine
    input_str = input("\nEnter a command: ").lower()
    match input_str.split():
        # global commands----------------------------overseer topic
        case ["stop"]:
            segmentMasterCommand("stop")
            print("Stopping the experiment")

        case ["start"]:
            # after the command is issued, it shall send T-5s to all segments
            millisec = int((time.time() * 1000) + countdown_ms)
            segmentMasterCommand("start::{}".format(millisec))
            print("Starting the experiment in {} seconds".format(
                countdown_ms / 1000))

        case ["reset"]:
            segmentMasterCommand("reset")
            print("Resetting all segments")

        case ["move", *args] if ('-p' in args and '-s' not in args):
            position = args[args.index('-p') + 1]
            if (int(position) > 255 or int(position) < 0):
                print("Invalid position")
            else:
                segmentMasterCommand("move::{}".format(position))
                print("Moving all segments")
        # all segment commands----------------------------segment topic

        case ["upload"]:
            print("Uploading data to all segments...")
            for segment_no in range(1, segment_count + 1):
                # programming technique that runs the upload and if it succeeds prints the message
                if segmentSendData(str(segment_no)):
                    print("✓ Upload success to segment # {}".format(segment_no))
                else:
                    print("✕ Upload failed to segment # {}".format(segment_no))
                time.sleep(0.250)
            # client.publish(overseerCommandPath, "upload::all")

        case ["timesync"]:
            print("Syncing time in all segments")
            # get the current time in UTC so there are no issues with timezones and daylight savings
            now_utc = datetime.now(timezone.utc)
            # client.publish(overseerCommandPath, "timesync::all", 1)

        case ["restart"]:
            print("Restarting time in all segments")
            for segment_no in range(1, segment_count + 1):
                # programming technique that runs the upload and if it succeeds prints the message
                if segment_restart(segment_no):
                    print("✓ Restart success  on segment # {}".format(segment_no))
                else:
                    print("✕ Restart failed on segment # {}".format(segment_no))
                time.sleep(0.250)

        case ["clear_pairing"]:
            print("Clearing all segment IDs")
            clearSegmentsID()

        # individual segment commands----------------------------segment topic

        case ["upload", *args] if '-s' in args:  # upload to a specific segment
            # get and verify the segment number validity
            segment_no = args[args.index('-s') + 1]
            if int(segment_no) > segment_count or int(segment_no) < 1:
                print("Invalid segment number")
            else:
                # programming technique that runs the upload and if it succeeds prints the message
                if segmentSendData(segment_no):
                    print("Uploaded data to segment # {}".format(segment_no))
                else:
                    print("Upload failed to segment # {}".format(segment_no))

        case ["move", *args] if '-s' and '-p' in args:  # move a specific segment
            segment_no = args[args.index('-s') + 1]
            position = args[args.index('-p') + 1]
            if move_segment(segment_no, position):
                print("Moving the segment # {} to {}".format(
                    segment_no, position))
            else:
                print("Move failed")

        case ["timesync", *args] if '-s' in args:  # timesync an individual segment
            segment_no = args[args.index('-s') + 1]
            # print("Syncing time in segment # {}".format(segment_no))

        case ["clear_pairing", *args] if '-s' in args:  # clear a specific segment ID
            segment_no = args[args.index('-s') + 1]
            if clearSegmentID(segment_no):
                print("Cleared the segment # {}".format(segment_no))
            else:
                print("Clearing failed")

        case ["reset", *args] if '-s' in args:  # reset a specific segment
            segment_no = args[args.index('-s') + 1]
            if segment_reset(segment_no):
                print("Reset the segment # {}".format(segment_no))
            else:
                print("Reset failed")

        case ["restart", *args] if '-s' in args:  # restart a specific segment
            segment_no = args[args.index('-s') + 1]
            if segment_restart(segment_no):
                print("Restarting the segment # {}".format(segment_no))
            else:
                print("Restart failed")

        case ["debug", *args] if '-s' in args:  # debug a specific segment
            segment_no = args[args.index('-s') + 1]
            print("Debugging segment # {}:".format(segment_no))
            print(getSegmentID(segment_no))
            print(get_segment_status(segment_no))

        case ["assign", *args] if '-s' in args:  # assign a segment an ID
            segment_no = args[args.index('-s') + 1]
            if addSegmentID(segment_no):
                print("Assignment successful")
            else:
                print("Assignment failed")

        # misc----------------------------

        case ["debug"]:
            print("Latest global message received: {}".format(
                latestOverseerReturnMessage))

        case ["exit" | "quit"]:
            print("Exiting the program")
            client.publish(overseerCommandPath, "stop", 1)
            client.loop_stop()
            quit()

        case _:
            print("Invalid command\nPlease refer to readme.md")
