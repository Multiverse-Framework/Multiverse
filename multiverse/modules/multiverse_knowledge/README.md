# Multiverse Knowledge

This module translates a scene graph in USD (Universal Scene Description) format into the knowledge graph in OWL format

## Usage

1. Create USD sublayer for semantic tagging

```bash
python3 scripts/TBox_to_usd.py 
        --in_owl=</path/to/TBox.owl>    # Input TBox ontology
        --out_usd=</path/to/TBox.usda>  # Output USD file representing TBox
```

2. Naive semantic tagging

```bash
python3 scripts/auto_sem_tag.py 
        --in_usd=</path/to/scene.usda>          # Input scene graph in USD
        --in_TBox_usd=</path/to/in_TBox.usda>   # Input USD file representing TBox
        --out_ABox_usd=</path/to/out_ABox.usda> # Output semantic scene graph in USD
```

3. Construct the knowledge graph

```bash
python3 scripts/usd_to_ABox.py
          --in_usd=</path/to/in_ABox.usda>      # Input semantic scene graph in USD
          --in_owl=</path/to/TBox.owl>          # Input upper level ontology
          --out_owl=</path/to/out_ABox.owl>     # Output knowledge graph of the scene
```

## Examples

Go to [`../../build/multiverse/modules/multiverse_knowledge`](multiverse_knowledge) and run `ctest`. The scene graph from [`../../tests/multiverse_knowledge/input`](multiverse_knowledge/input) will be translated in the `output` folder in [`../../tests/multiverse_knowledge`](multiverse_knowledge)