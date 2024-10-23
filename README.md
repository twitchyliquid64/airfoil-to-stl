# Airfoil-to-STL

Converts airfoil data into an STL file for printing or further modification.

```
Simple program to generate a wing model given airfoil parameters

Usage: airfoil-to-stl [OPTIONS] --semi-wingspan <SEMI_WINGSPAN> --root-chord <ROOT_CHORD> --tip-chord <TIP_CHORD> --outfile <OUTFILE> <FILE>

Arguments:
  <FILE>  Selig-formatted airfoil data

Options:
  -w, --semi-wingspan <SEMI_WINGSPAN>  Width of each wing
  -s, --sweep <SWEEP>                  Distance to sweep the wing back [default: 0]
  -r, --root-chord <ROOT_CHORD>        Root chord length
  -t, --tip-chord <TIP_CHORD>          Tip chord length
  -o, --outfile <OUTFILE>              Where to write the stl-formatted model
  -h, --help                           Print help
  -V, --version                        Print version
```

It takes Selig-formatted airfoil data (for instance, downloaded from http://airfoiltools.com/) and generates an STL based on the input parameters.


## Thanks

Big thanks to @Markk116 for [helping out](https://github.com/ricosjp/truck/issues/58#issuecomment-2371474180) with some of the modelling.
