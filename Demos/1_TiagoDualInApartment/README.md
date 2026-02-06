### 9️⃣ Download MuJoCo 3.4.0 and Copy Plugins

```bash
export MUJOCO_VERSION=3.4.0
wget -qO- https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz | tar -xz -C $PWD/Multiverse/Demos/1_TiagoDualInApartment/
cp -f ./Multiverse/MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/*.so \
      ./Multiverse/Demos/1_TiagoDualInApartment/mujoco-${MUJOCO_VERSION}/bin/mujoco_plugin/
```
