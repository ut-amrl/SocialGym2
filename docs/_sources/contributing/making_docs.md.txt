#  Making docs

The source code for all the documentation can be found in `{PROJECT_ROOT}/docs_src`

We use [Sphinx](https://www.sphinx-doc.org/en/master/) to create our documentation and host it in Github Pages.

Check out `docs_src/index.rst` for the layout of the pages (starting on line 6).  That file will point to the source doc
for each of the pages on the site.  Everything is written in markdown currently, so when you find the `.md` file, you 
can add/change/create new pages in markdown.


Creating a new doc requires that you make a `filename.md` and then point to it in the `index.rst` file (similar to the
previous docs).

You can use `./auto_build.sh` to see your changes to the documentation site live (it will update your local version
as you save files.) The service is usually hosted on http://127.0.0.1:8000 but it will also tell you where the local 
version of the site is.

### Uploading changes

#### 1.) Run `./build.sh`

#### 2.) Run `./git_add.sh`

#### 3.) The files are now staged in git, you can do 
```shell
git commit -m "Doc changes"
git push
```
and the website should update within the next 5-10minutes.
