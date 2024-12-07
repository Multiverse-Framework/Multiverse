# Multiverse Connector plugin

## Multiverse Connector

You can use it like:

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.multiverse_connector">
      <instance name="mujoco_client">
        <config key="host" value="tcp://127.0.0.1"/>
        <config key="server_port" value="7000"/>
        <config key="client_port" value="7500"/>
        <config key="world_name" value="world"/>
        <config key="simulation_name" value="mujoco_sim"/>
        <config key="send" value="{'world': ['position', 'quaternion']}" />
      </instance>
    </plugin>
  </extension>

  <worldbody/>
</mujoco>
```
