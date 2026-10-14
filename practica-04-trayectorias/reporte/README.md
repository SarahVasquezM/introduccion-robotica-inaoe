# IEEE report

The manuscript is `report.tex`. Its figures and numeric macros are generated
from the successful Gazebo run archived in `../resultados/detour_26s`.

Regenerate the evidence figures and metrics from the
`practica-04-trayectorias` directory:

```bash
python3 src/practica4_rrt/scripts/generate_report_figures.py \
  resultados/detour_26s --output reporte/figures
```

On Ubuntu, install the IEEE class and recommended Times-compatible fonts once:

```bash
sudo apt install texlive-publishers texlive-fonts-recommended
```

Then compile the article from this directory:

```bash
pdflatex report.tex
pdflatex report.tex
```

The second pass resolves cross-references. The plots are included as vector PDF
files; PNG copies are retained for quick inspection.
