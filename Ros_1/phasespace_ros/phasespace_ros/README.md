# phasespace_ros

To add this package to your workspace, put this inside the `src/` folder. 
You may use one of

- `git clone`
- `git subtree`
- `git submodule`
- rosinstall

to acheive this


To run the tracker, run

```console
$ roslaunch phasespace phasespace.launch tracker_filename:=<path to tracker JSON file>
```

See the [documentation](./phasespace_docs.pdf) for instructions on producing a 
tracker JSON file.

Once running, this package will publish tracked objects to `/tf` as well as
`/phasespace/rigids`. The location of each marker can also be retrieved from the
`/phasespace/markers` topic.
