// Package webstatic embeds the UI directly from its source directory.
package webstatic

import "embed"

//go:embed index.html guide.html favicon.ico css js img fonts
var assets embed.FS

func FS() embed.FS { return assets }
