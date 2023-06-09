# Universim Knowledge

This package offers functionality to translate USD (Universal Scene Description) files into knowledge graphs using the OWL (Web Ontology Language) format.

## Usage

### Construct USD sublayer for semantic tagging (TBox_owl -> TBox_usd)

```bash
rosrun universim_knowledge TBox_to_usd.py <in_TBox_onto.owl> <out_TBox_usd.usda>
```

### Auto semmantic tagging (only works for known names) (non_semantic_usd + TBox_usd -> ABox_usd)

```bash
rosrun universim_knowledge auto_sem_tag.py <in_ABox_usd.usda> <in_TBox_usd.usda> <out_ABox_usd.usda>
```

### Construct knowledge graph (ABox_usd + TBox_owl -> ABox_owl)

```bash
rosrun universim_knowledge usd_to_ABox.py <in_ABox_usd.usda> <in_TBox_onto.owl> <out_ABox_onto.owl>
```

## Examples

```bash
roslaunch universim_parser usd_to_KG.launch
```
