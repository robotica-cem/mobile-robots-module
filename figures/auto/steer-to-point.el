(TeX-add-style-hook
 "steer-to-point"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("standalone" "margin=5mm")))
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("fontenc" "T1") ("inputenc" "utf8")))
   (TeX-run-style-hooks
    "latex2e"
    "standalone"
    "standalone10"
    "fontenc"
    "inputenc"
    "pgf"
    "tikz"
    "pgfplots")
   (TeX-add-symbols
    '("gettikzxy" 3)
    "pxx"
    "pyy"))
 :latex)

