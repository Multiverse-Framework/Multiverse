import json
from typing import Dict, Any


def str_to_dict(data: str) -> Dict[str, Any]:
    if data is None:
        return {}
    try:
        return json.loads(data.replace("'", '"'))
    except json.JSONDecodeError as e:
        print(f"Failed to parse {data}: {str(e)}")
        return {}
