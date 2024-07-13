#!/usr/bin/python3
# -*- coding: utf-8 -*-

# import context  # Ensures paho is in PYTHONPATH
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import json
import logging
import random
import datetime
import time
logging.basicConfig(level=logging.DEBUG)

p_hostname = "emqx"  # "localhost"
p_auth = {'username': "admin", 'password': "Ignition1!"}
p_client_id = "paho-dt-update"
p_base_topic = "mab/columbia/brewing"


def pub_multi(msgs):
    publish.multiple(msgs, hostname=p_hostname, client_id=p_client_id,
                     auth=p_auth)


def pub_single(topic, payload):
    publish.single(topic, payload, hostname=p_hostname, client_id=p_client_id,
                   auth=p_auth)


def on_phase_change(client, userdata, message):
    # Assuming the payload is a simple string indicating the phase
    phase_value = message.payload.decode()

    # Log the received phase change
    # print(f"Received phase change: {phase_value}")
    # print(f"Type of phase change: {type(phase_value)}")
    # Example: Publish to different topics based on the phase value
    if phase_value == "1":
        phase_1()
    if phase_value == "2":
        phase_2()
    if phase_value == "3":
        phase_3()
    if phase_value == "4":
        phase_4()
    if phase_value == "5":
        phase_5()
    if phase_value == "6":
        phase_6()


def phase_0():
    '''
    #SETUP THE CUSTOMERS EXAMPLE PAYLOAD heirarchy
    '''
    json_payload = {"flow": 19123.909123, "feedback": 1.0256, "cmd": 1.124}
    payload_str = json.dumps(json_payload)
    base_msgs = [
        {'topic': p_base_topic + "/line1/edge", 'payload': payload_str},
        {'topic': p_base_topic + "/line2/edge", 'payload': payload_str},
        {'topic': p_base_topic + "/line3/edge", 'payload': payload_str},
        {'topic': p_base_topic + "/line4/edge", 'payload': payload_str},
        {'topic': "dt_phase", 'payload': 0}
    ]
    pub_multi(base_msgs)


def phase_1():
    '''
    publish MES KPI data to all lines
    '''
    mes_json = {"oee": 0.94684, "availability": 0.999,
                "performance": 0.999, "quality": 0.999}
    mes_payload = json.dumps(mes_json)
    msgs = [
        {'topic': p_base_topic + "/line1/mes/kpi", 'payload': mes_payload}
    ]
    pub_multi(msgs)


def phase_2():
    '''
    #SETUP THE CUSTOMERS EXAMPLE PAYLOAD heirarchy
    '''
    mes_json = {
        "line_id": "Line1",
        "current_status": "Running",
        "performance_metrics": {
            "oee": 85.5,
            "availability": 90.0,
            "performance": 95.0,
            "quality": 98.0
        },
        "current_work_orders": [
            {
                "work_order_id": "WO001",
                "product_id": "ProductA",
                "target_quantity": 1000,
                "produced_quantity": 800,
                "defective_quantity": 20,
                "start_time": "2024-07-10T08:00:00Z",
                "end_time": "2024-07-10T16:00:00Z",
                "status": "In Progress",
                "operations": [
                    {
                        "operation_id": "Op1",
                        "description": "Assembly",
                        "status": "Completed",
                        "start_time": "2024-07-10T08:00:00Z",
                        "end_time": "2024-07-10T12:00:00Z",
                        "metrics": {
                            "cycle_time": 45,
                            "downtime": 5
                        }
                    },
                    {
                        "operation_id": "Op2",
                        "description": "Quality Check",
                        "status": "In Progress",
                        "start_time": "2024-07-10T12:30:00Z",
                        "end_time": "2024-07-10T14:30:00Z",
                        "metrics": {
                            "inspection_time": 20,
                            "defective_items": 2
                        }
                    }
                ]
            }
        ]
    }
    mes_payload = json.dumps(mes_json)
    msgs = [
        {'topic': p_base_topic + "/line1/mes/kpi", 'payload': mes_payload},
        {'topic': p_base_topic + "/line2/mes/kpi", 'payload': mes_payload},
        {'topic': p_base_topic + "/line3/mes/kpi", 'payload': mes_payload},
        {'topic': p_base_topic + "/line4/mes/kpi", 'payload': mes_payload}
    ]
    pub_multi(msgs)


def phase_3():
    '''
    #SETUP THE CUSTOMERS EXAMPLE PAYLOAD heirarchy
    '''
    erp_json = {
        "production_orders": [
            {
                "order_id": "PO001",
                "product_id": "ProductA",
                "quantity": 1000,
                "status": "In Progress",
                "start_date": "2024-07-10",
                "end_date": "2024-07-20",
                "production_line": "Line1",
                "work_orders": [
                    {
                        "work_order_id": "WO001",
                        "operation": "Assembly",
                        "status": "Completed",
                        "scheduled_start_time": "2024-07-10T08:00:00Z",
                        "scheduled_end_time": "2024-07-10T12:00:00Z",
                        "actual_start_time": "2024-07-10T08:00:00Z",
                        "actual_end_time": "2024-07-10T11:45:00Z",
                        "resources": [
                            {
                                "resource_id": "MachineA",
                                "resource_type": "Machine",
                                "status": "Available"
                            },
                            {
                                "resource_id": "Operator1",
                                "resource_type": "Operator",
                                "status": "Assigned"
                            }
                        ]
                    },
                    {
                        "work_order_id": "WO002",
                        "operation": "Quality Check",
                        "status": "Scheduled",
                        "scheduled_start_time": "2024-07-10T13:00:00Z",
                        "scheduled_end_time": "2024-07-10T15:00:00Z",
                        "resources": [
                            {
                                "resource_id": "Inspector1",
                                "resource_type": "Inspector",
                                "status": "Assigned"
                            }
                        ]
                    }
                ]
            }
        ]
    }
    erp_payload = json.dumps(erp_json)
    msgs = [
        {'topic': p_base_topic + "/line1/erp/workorders", 'payload': erp_payload},
        {'topic': p_base_topic + "/line2/erp/workorders", 'payload': erp_payload},
        {'topic': p_base_topic + "/line3/erp/workorders", 'payload': erp_payload},
        {'topic': p_base_topic + "/line4/erp/workorders", 'payload': erp_payload}
    ]
    pub_multi(msgs)


def phase_4():
    '''
    #SETUP THE CUSTOMERS EXAMPLE PAYLOAD heirarchy
    '''
    cmms_json = {
        "assets": [
            {
                "asset_id": "Asset001",
                "type": "Pump",
                "location": "Plant A",
                "manufacturer": "PumpCo",
                "model": "P2000",
                "serial_number": "SN12345",
                "installation_date": "2020-01-15",
                "maintenance_plans": [
                    {
                        "plan_id": "Plan001",
                        "description": "Routine Inspection",
                        "frequency": "Monthly",
                        "tasks": [
                            {
                                "task_id": "Task001",
                                "description": "Check oil level",
                                "estimated_duration": "30 minutes"
                            },
                            {
                                "task_id": "Task002",
                                "description": "Inspect seals",
                                "estimated_duration": "15 minutes"
                            }
                        ]
                    }
                ],
                "maintenance_records": [
                    {
                        "record_id": "Record001",
                        "date": "2023-06-15",
                        "tasks_completed": [
                            {
                                "task_id": "Task001",
                                "status": "Completed",
                                "comments": "Oil level normal"
                            },
                            {
                                "task_id": "Task002",
                                "status": "Completed",
                                "comments": "Seals in good condition"
                            }
                        ],
                        "performed_by": "Technician A"
                    }
                ]
            }
        ],
        "work_orders": [
            {
                "work_order_id": "WO001",
                "asset_id": "Asset001",
                "description": "Repair leaking seal",
                "priority": "High",
                "date_created": "2024-07-10",
                "due_date": "2024-07-12",
                "status": "Open",
                "tasks": [
                    {
                        "task_id": "Task003",
                        "description": "Replace seal",
                        "estimated_duration": "1 hour"
                    }
                ],
                "assigned_to": "Technician B"
            }
        ]
    }
    cmms_payload = json.dumps(cmms_json)
    msgs = [
        {'topic': p_base_topic + "/line1/cmms", 'payload': cmms_payload},
        {'topic': p_base_topic + "/line2/cmms", 'payload': cmms_payload},
        {'topic': p_base_topic + "/line3/cmms", 'payload': cmms_payload},
        {'topic': p_base_topic + "/line4/cmms", 'payload': cmms_payload}
    ]
    pub_multi(msgs)


def phase_5():
    '''
    #SETUP THE CUSTOMERS EXAMPLE PAYLOAD heirarchy
    '''
    ISA88_json = {
        "process_cell": {
            "cell_id": "Cell1",
            "batches": [
                {
                    "batch_id": "Batch1",
                    "units": [
                        {
                            "unit_id": "Unit1",
                            "unit_procedures": [
                                {
                                    "procedure_id": "Procedure1",
                                    "operations": [
                                        {
                                            "operation_id": "Op1",
                                            "phases": [
                                                {
                                                    "phase_id": "Phase1",
                                                    "description": "Heating",
                                                    "start_time": "2024-07-12T08:00:00Z",
                                                    "end_time": "2024-07-12T09:00:00Z",
                                                    "parameters": {
                                                        "temperature": 75,
                                                        "pressure": 1.2
                                                    }
                                                },
                                                {
                                                    "phase_id": "Phase2",
                                                    "description": "Mixing",
                                                    "start_time": "2024-07-12T09:00:00Z",
                                                    "end_time": "2024-07-12T10:00:00Z",
                                                    "parameters": {
                                                        "stir_speed": 100,
                                                        "duration": 3600
                                                    }
                                                }
                                            ]
                                        }
                                    ]
                                }
                            ]
                        },
                        {
                            "unit_id": "Unit2",
                            "unit_procedures": [
                                {
                                    "procedure_id": "Procedure2",
                                    "operations": [
                                        {
                                            "operation_id": "Op2",
                                            "phases": [
                                                {
                                                    "phase_id": "Phase3",
                                                    "description": "Cooling",
                                                    "start_time": "2024-07-12T10:00:00Z",
                                                    "end_time": "2024-07-12T11:00:00Z",
                                                    "parameters": {
                                                        "cooling_rate": 5,
                                                        "pressure": 0.8
                                                    }
                                                }
                                            ]
                                        }
                                    ]
                                }
                            ]
                        }
                    ]
                }
            ]
        }
    }
    ISA88_payload = json.dumps(ISA88_json)
    msgs = [
        {'topic': p_base_topic + "/line1/ISA88", 'payload': ISA88_payload},
        {'topic': p_base_topic + "/line2/ISA88", 'payload': ISA88_payload},
        {'topic': p_base_topic + "/line3/ISA88", 'payload': ISA88_payload},
        {'topic': p_base_topic + "/line4/ISA88", 'payload': ISA88_payload}
    ]
    pub_multi(msgs)


def phase_6():
    '''
    #SETUP THE CUSTOMERS EXAMPLE PAYLOAD heirarchy
    '''
    ISO5501_json = {
        "production_line": {
            "line_id": "Line1",
            "context": {
                "external_issues": "Market conditions, regulatory requirements",
                "internal_issues": "Resource availability, organizational structure",
                "interested_parties": {
                    "customers": "Quality products, timely delivery",
                    "regulators": "Compliance with laws and regulations",
                    "employees": "Safe working conditions, career growth"
                }
            },
            "leadership": {
                "line_manager": "John Doe",
                "roles_responsibilities": {
                    "asset_manager": "Jane Smith",
                    "maintenance_supervisor": "Tom Brown"
                },
                "communication": "Regular meetings, email updates"
            },
            "planning": {
                "asset_management_objectives": [
                    "Increase asset utilization by 10% over the next year",
                    "Reduce maintenance costs by 15% in two years"
                ],
                "risk_management": "Risk assessment performed quarterly"
            },
            "support": {
                "resources": {
                    "financial": "Annual budget of $1M for maintenance",
                    "human": "10 maintenance technicians",
                    "infrastructure": "Spare parts inventory, maintenance tools"
                },
                "competence": "Training programs for staff",
                "communication": "Internal newsletter, team meetings",
                "information_management": {
                    "asset_register": "Centralized database",
                    "documentation": "Maintenance manuals, SOPs"
                }
            },
            "operation": {
                "maintenance_activities": [
                    {
                        "activity_id": "MA1",
                        "description": "Routine inspection",
                        "frequency": "Monthly",
                        "responsible_person": "Tom Brown"
                    },
                    {
                        "activity_id": "MA2",
                        "description": "Lubrication of machinery",
                        "frequency": "Quarterly",
                        "responsible_person": "Jane Smith"
                    }
                ],
                "life_cycle_management": "Asset life cycle monitored and recorded",
                "outsourced_processes": "Third-party calibration services"
            },
            "performance_evaluation": {
                "monitoring": "Real-time asset monitoring system",
                "internal_audits": "Annual internal audit",
                "management_review": "Bi-annual review meetings"
            },
            "improvement": {
                "nonconformity_management": "Nonconformities logged and addressed",
                "continuous_improvement": "Kaizen events, employee suggestions"
            }
        }
    }

    ISO5501_payload = json.dumps(ISO5501_json)
    msgs = [
        {'topic': p_base_topic + "/line1/ISO5501", 'payload': ISO5501_payload},
        {'topic': p_base_topic + "/line2/ISO5501", 'payload': ISO5501_payload},
        {'topic': p_base_topic + "/line3/ISO5501", 'payload': ISO5501_payload},
        {'topic': p_base_topic + "/line4/ISO5501", 'payload': ISO5501_payload}
    ]
    pub_multi(msgs)


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected successfully.")
        # set_phase_0()
    else:
        print(f"Failed to connect, return code {rc}")


# mqttc = mqtt.Client()
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
# Set the username and password
mqttc.username_pw_set(username="admin", password="Ignition1!")
mqttc.enable_logger(logger=logging.getLogger(__name__))
mqttc.message_callback_add("dt_phase", on_phase_change)
mqttc.on_connect = on_connect
mqttc.connect(host=p_hostname)
mqttc.subscribe("$SYS/#", 0)
mqttc.subscribe("dt_phase/#", 0)

phase_0()

mqttc.loop_forever()
