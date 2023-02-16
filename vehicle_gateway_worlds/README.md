It is convenient to annotate the world SDF with pre-defined spawn positions. This can be done using `frame` tags in SDF, which are children of the `world` tag, like this:

```xml
    <frame name="pad_1">
      <pose>-9.7948 -8.31848 2.0 0.0 0.0 0.0</pose>
    </frame>
```

The ordering is (x, y, z, roll, pitch, yaw), as defined in the [SDF documentation](http://sdformat.org/spec?ver=1.9&elem=world#world_frame) documentation.
