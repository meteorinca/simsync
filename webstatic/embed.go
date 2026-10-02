// Package webstatic embeds the SimSync web dashboard files.
package webstatic

import (
	"embed"
	"io/fs"
	"log"
)

//go:embed all:web
var root embed.FS

// FS returns the embedded web/ directory as an fs.FS suitable for http.FileServer.
func FS() fs.FS {
	sub, err := fs.Sub(root, "web")
	if err != nil {
		log.Fatalf("[webstatic] embed sub error: %v", err)
	}
	return sub
}
