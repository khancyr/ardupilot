import os
CURR_DIR = os.getcwd()
print(CURR_DIR)

json_filepath = CURR_DIR + "/test.json"

vehicle = "copter"
frame = None
board = "testboard"
text = 1234
data = 5678

vehicle2 = "rover"
frame2 = None
board2 = "testboard2"
text2 = 91234
data2 = 95678

vehicle3 = "plae"

# summary_data_list2 = [{
#     board : {vehicle : {"text" : text, "data" : data}},
#     "testboard2": {vehicle : {"text" : text, "data" : data}},
#     "testboard3": {vehicle : {"text" : text, "data" : data}, vehicle2 : {"text" : text2, "data" : data2}},
# }
# ]
summary_data_list = [{
    board2: {vehicle : {"text" : text, "data" : data}},
}
]

summary_data_list2 = [{
    board: {vehicle3 : {"text" : text2, "data" : data2}},
}
]

summary_data_list3 = [{
    "testbpard3": {vehicle2 : {"text" : text2, "data" : data2}},
}
]

test_list = summary_data_list

print("prout")
# check file exist to prevent json failure
if os.path.isfile(json_filepath):
    json_exist = True
    print("exist")
else:
    json_exist = False
    print("don't exist")

if json_filepath is not None:
    import json
    if json_exist:
        main_data_summary = None
        with open(json_filepath, "r+") as summary_json:
            import json
            main_data_summary = json.load(summary_json)
            print(main_data_summary)
            key= next(iter(test_list[0]))
            print(key)
            if key in main_data_summary[0]:
                print("prout")
                main_data_summary[0][key].update(test_list[0][key])
                print(main_data_summary)
            else:
                print("not prout")
                main_data_summary[0].update(test_list[0])

        with open(json_filepath, "w") as summary_json:
            summary_data_json = json.dumps(main_data_summary)
            summary_json.write(summary_data_json)
    else:
        with open(json_filepath, "a") as summary_json:
            output_json = test_list

            summary_data_json = json.dumps(output_json)
            summary_json.write(summary_data_json)
