import json


def parse_report(json_file):
    with open(json_file, "r", encoding="utf-8") as f:
        js = dict()
        js = json.loads(f.read())
        summary: dict
        summary = js.get("summary", dict())
        print("Passed:", summary.get("passed"))
        print("Failed:", summary.get("failed", 0))
        print("Total:", summary.get("num_tests"))


if __name__ == "__main__":
    parse_report("report.json")
