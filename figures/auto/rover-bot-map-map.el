(TeX-add-style-hook
 "rover-bot-map-map"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("standalone" "margin=0mm")))
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
    "utfsym")
   (TeX-add-symbols
    '("wall" 2)
    "wheel"
    "robot"
    "wthickness"))
 :latex)

